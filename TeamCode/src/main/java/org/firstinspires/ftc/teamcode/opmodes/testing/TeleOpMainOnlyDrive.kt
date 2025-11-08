package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.HardwareMap
import dev.nextftc.core.commands.Command
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.hardware.driving.MecanumDriverControlled
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem
import org.firstinspires.ftc.teamcode.subsystems.LowerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.MagblockServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.TurretSubsystem

class TeleOpMainOnlyDrive: NextFTCOpMode() {

    private val frontLeftMotor = MotorEx("frontLeft");
    private val frontRightMotor = MotorEx("frontRight");
    private val backLeftMotor = MotorEx("backLeft");
    private val backRightMotor = MotorEx("backRight");



    override fun onStartButtonPressed(): Unit {
//        IntakeSubsystem.DriverCommandDefaultOn(
//            Gamepads.gamepad1.leftTrigger,
//        ).schedule();

        MecanumDriverControlled(
            frontLeftMotor,
            frontRightMotor,
            backLeftMotor,
            backRightMotor,
            Gamepads.gamepad1.leftStickY,
            Gamepads.gamepad1.leftStickX,
            Gamepads.gamepad1.rightStickX,
        ).schedule();



    }


}
