package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.localization.OdometrySubsystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "AutoTest", group = "Testing")
public class LocalTest extends NextFTCOpMode {

    private OdometrySubsystem odom;
    private final MotorEx frontLeftMotor = new MotorEx("front_left").reversed();
    private final MotorEx frontRightMotor = new MotorEx("front_right");
    private final MotorEx backLeftMotor = new MotorEx("back_left").reversed();
    private final MotorEx backRightMotor = new MotorEx("back_right");

    @Override
    public void onStartButtonPressed() {
        Command driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
        );
        driverControlled.schedule();
    }


    @Override
    public void runOpMode(){
        telemetry.addLine("Initializing Odometry Subsystem...");
        telemetry.update();



        try {
            odom = new OdometrySubsystem(0, 0, 0, hardwareMap);
            telemetry.addLine("Odometry initialization successful!");
            telemetry.update();
        } catch (Exception e) {
            telemetry.addLine("Error initializing Odometry: " + e.getMessage());
            telemetry.update();
            return; // stop here to prevent crash
        }

        telemetry.addLine("Press PLAY to start tracking...");
        telemetry.update();

        waitForStart();

        // Loop while OpMode is active
        while (opModeIsActive()) {
            odom.updateOdom();

            telemetry.addData("x (inch)", odom.getROx1());
            telemetry.addData("y (inch)", odom.getROy1());
            telemetry.addData("h (radians)", odom.getROh());
            telemetry.addData("distance", odom.getDistance());
            telemetry.update();

            sleep(50); // update rate ~20Hz
        }
    }
}
