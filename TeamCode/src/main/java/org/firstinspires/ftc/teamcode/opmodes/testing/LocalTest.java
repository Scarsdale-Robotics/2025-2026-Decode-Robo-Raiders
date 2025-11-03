package org.firstinspires.ftc.teamcode.opmodes.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.subsystems.localization.OdometrySubsystem;

@TeleOp(name = "AutoTest", group = "Testing")
public class LocalTest extends LinearOpMode {

    private OdometrySubsystem odom;


    @Override
    public void runOpMode() throws InterruptedException {
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
