package org.firstinspires.ftc.teamcode.opmodes.show

import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.Angle
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.LowerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagblockServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem

class ShowTeleOp : NextFTCOpMode() {

    init {
        addComponents(
            SubsystemComponent(
                LowerSubsystem,
                OuttakeSubsystem
            ),
            PedroComponent(Constants::createFollower),
            BulkReadComponent,
            BindingsComponent
        )
    }

    var lowerOverridePower = 0.0;
    override fun onStartButtonPressed() {
        MagblockServoSubsystem.unblock()
        MagblockServoSubsystem.block()

        val lowerMotorDrive = MagMotorSubsystem.DriverCommand(
            Gamepads.gamepad1.rightTrigger,
            Gamepads.gamepad1.leftTrigger,
            { lowerOverridePower }
        );
        lowerMotorDrive();

        val intakeMotorDrive = IntakeMotorSubsystem.DriverCommand(
            Gamepads.gamepad1.rightTrigger,
            Gamepads.gamepad1.leftTrigger,
            { lowerOverridePower }
        )
        intakeMotorDrive()

        Gamepads.gamepad1.circle whenBecomesTrue {
            lowerOverridePower = 1.0;
            MagblockServoSubsystem.unblock()
            ShooterSubsystem.isShooting = true  // todo: tell aaron to set this (nvm)
        } whenBecomesFalse {
            lowerOverridePower = 0.0;
            MagblockServoSubsystem.block()
            ShooterSubsystem.isShooting = false
        }
    }

    override fun distanceToVelocity(dist: Double): Double {

    }

    override fun distAndVeloToTheta(dist: Double, velo: Double): Angle {

    }

    var lastRuntime = 0.0
    var dxyp = 20.0
    override fun onUpdate() {
        telemetry.addData("Loop Time (ms)", runtime - lastRuntime);
        lastRuntime = runtime;

        telemetry.addData("Goal Dist (in), dxyp [DPAD UP/DOWN TO ADJUST]", dxyp);

        ShooterSubsystem.AutoAim(
            dxyp,
            { distanceToVelocity(it) }
        )()
        TurretThetaSubsystem.AutoAim(
            dxyp,
            { distAndVeloToTheta(dxyp, ShooterSubsystem.velocity) },
        )()

        telemetry.update()
    }
}