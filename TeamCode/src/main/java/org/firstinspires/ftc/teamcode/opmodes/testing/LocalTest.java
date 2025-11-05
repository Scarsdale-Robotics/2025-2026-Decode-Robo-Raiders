package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.localization.OdometrySubsystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "AutoTest", group = "Testing")
public class LocalTest extends NextFTCOpMode {

    private OdometrySubsystem odom;
    private MecanumDriverControlled driverControlled;

    // Motors
    private MotorEx frontLeftMotor;
    private MotorEx frontRightMotor;
    private MotorEx backLeftMotor;
    private MotorEx backRightMotor;


    public void initHardware() {
        // Initialize motors
        frontLeftMotor = new MotorEx("FL").reversed().zeroed().brakeMode();
        frontRightMotor = new MotorEx("FR").zeroed().brakeMode();
        backLeftMotor = new MotorEx("BL").reversed().zeroed().brakeMode();
        backRightMotor = new MotorEx("BR").zeroed().brakeMode();

        // Create robot-centric mecanum driver
        driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY().negate(), // forward/back
                Gamepads.gamepad1().leftStickX(),          // strafe
                Gamepads.gamepad1().rightStickX()          // turn
        );

        // Schedule driver control
        driverControlled.schedule();
    }

    @Override
    public void runOpMode() {
        // Initialize odometry
        telemetry.addLine("Initializing Odometry Subsystem...");
        telemetry.update();
        this.initHardware();

        try {
            odom = new OdometrySubsystem(0, 0, 0, hardwareMap);
            telemetry.addLine("Odometry initialized successfully!");
        } catch (Exception e) {
            telemetry.addLine("Odometry initialization failed: " + e.getMessage());
            telemetry.update();
            return;
        }

        telemetry.addLine("Press PLAY to start driving...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update driver (reads joystick values automatically)
            driverControlled.update();

            // Update odometry
            odom.updateOdom();

            // Telemetry
            telemetry.addData("x (inch)", odom.getROx1());
            telemetry.addData("y (inch)", odom.getROy1());
            telemetry.addData("h (radians)", odom.getROh());
            telemetry.addData("distance", odom.getDistance());
            telemetry.update();

            // Loop at ~20Hz
            sleep(50);
        }
    }
}
