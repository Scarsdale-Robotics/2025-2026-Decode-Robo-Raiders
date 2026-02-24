package org.firstinspires.ftc.teamcode.opmodes.show

import com.pedropathing.geometry.Pose
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.rad
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
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem
import kotlin.math.PI
import kotlin.math.hypot

class ShowTeleOp : NextFTCOpMode() {

    val h:  Angle  get() { return (PedroComponent.follower.pose.heading).rad;}

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

    override fun onInit() {
        PedroComponent.follower.pose = Pose(0.0, 0.0, -PI / 2)
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
    var dyp = 20.0;
    var dxp = 0.0;
    override fun onUpdate() {
        telemetry.addData("Loop Time (ms)", runtime - lastRuntime);
        lastRuntime = runtime;

        telemetry.addData("dyp [DPAD U/D TO ADJUST]", dyp);
        telemetry.addData("dxp [DPAD L/R TO ADJUST]", dxp);

        val dxyp = hypot(dxp, dyp);

        TurretPhiSubsystem.AutoAim(
            dxp,
            dyp,
            h
        )
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