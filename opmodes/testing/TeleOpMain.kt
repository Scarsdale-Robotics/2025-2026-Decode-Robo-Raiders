package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.hardware.Gamepad
import dev.nextftc.core.commands.Command
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.hardware.driving.MecanumDriverControlled
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.subsystems.MagazineSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.intake.IntakeDriverCommand
import org.firstinspires.ftc.teamcode.subsystems.magazine.MagblockServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.TurretSubsystem

class TeleOpMain : NextFTCOpMode() {
    private val intakeMotor = MotorEx("intake");

    private val frontLeftMotor = MotorEx("frontLeft");
    private val frontRightMotor = MotorEx("frontRight");
    private val backLeftMotor = MotorEx("backLeft");
    private val backRightMotor = MotorEx("backRight");

    private var autoAimEnabled = true;
    private val autoAimCommand: Command;

    init {
        addComponents(
            SubsystemComponent(
                OuttakeSubsystem,
                MagazineSubsystem
            )  // todo: consider add localization subsystem
        )

        autoAimCommand = TurretSubsystem.AutoAim();  // todo: connect with localization
    }

    override fun onStartButtonPressed() {
        IntakeDriverCommand(
            intakeMotor,
            Gamepads.gamepad1.rightTrigger,
            Gamepads.gamepad1.leftTrigger,
        ).schedule();

        MecanumDriverControlled(
            frontLeftMotor,
            frontRightMotor,
            backLeftMotor,
            backRightMotor,
            Gamepads.gamepad1.leftStickY,
            Gamepads.gamepad1.leftStickX,
            Gamepads.gamepad1.rightStickX,
        ).schedule();

        // auto-aim
        autoAimCommand.schedule();
    }

    override fun onUpdate() {
        // circle --> shoot
        Gamepads.gamepad1.circle whenBecomesTrue MagblockServoSubsystem.open;
        Gamepads.gamepad1.circle whenBecomesFalse MagblockServoSubsystem.close;

        // g2 both bumpers + circle --> toggle autoAim
        Gamepads.gamepad2.leftBumper and Gamepads.gamepad2.rightBumper and Gamepads.gamepad2.circle whenBecomesTrue {
            if (!autoAimEnabled) {
                autoAimCommand.cancel();
            } else {
                autoAimCommand.schedule();
            }
            autoAimEnabled = !autoAimEnabled;
        }

        // indicators
        if (!autoAimEnabled) {
            gamepad2.setLedColor(
                255.0, 0.0, 0.0, Gamepad.LED_DURATION_CONTINUOUS);
        } else {
            gamepad2.setLedColor(
                255.0, 255.0, 255.0, Gamepad.LED_DURATION_CONTINUOUS);
        }
    }
}