package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.pedropathing.follower.Follower
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.opmodes.testing.baseSubsystems.ShooterFTest.Companion.speed
import org.firstinspires.ftc.teamcode.subsystems.LowerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.LowerMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagblockServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem

class BasicTeleOp(): NextFTCOpMode() {

    companion object {
        @JvmField var speed1 = 0.0;
    }

    override fun onInit() {
        addComponents(
            SubsystemComponent(
                LowerSubsystem,
                OuttakeSubsystem
            ),
            BindingsComponent,
            BulkReadComponent
        )

        ShooterSubsystem.off()
        LowerMotorSubsystem.off()
        MagServoSubsystem.stop()
    }

    override fun onStartButtonPressed() {


        val driveCommand = PedroDriverControlled(
            Gamepads.gamepad1.leftStickY.map {it},
            Gamepads.gamepad1.leftStickX.map {it},
            Gamepads.gamepad1.rightStickX.map {it},
            false
        )
        driveCommand();

        val lowerMotorDrive = LowerMotorSubsystem.DriverCommandDefaultOn(
            Gamepads.gamepad1.leftTrigger
        );
        lowerMotorDrive();

        val magDrive = MagServoSubsystem.DriverCommand(
            Gamepads.gamepad1.leftTrigger.greaterThan(0.0)
        )
        magDrive();

        ShooterSubsystem.On(speed1)();

        MagblockServoSubsystem.unblock()
        Gamepads.gamepad1.circle
            .whenTrue { MagblockServoSubsystem.unblock }
            .whenBecomesFalse { MagblockServoSubsystem.block }

    }

}
