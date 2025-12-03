//package org.firstinspires.ftc.teamcode.Auton;
//
//import static org.firstinspires.ftc.teamcode.PedroPathing.Tuning.changes;
//import static org.firstinspires.ftc.teamcode.PedroPathing.Tuning.drawCurrent;
//import static org.firstinspires.ftc.teamcode.PedroPathing.Tuning.drawCurrentAndHistory;
//import static org.firstinspires.ftc.teamcode.PedroPathing.Tuning.stopRobot;
//import static org.firstinspires.ftc.teamcode.PedroPathing.Tuning.telemetryM;
//import static dev.nextftc.extensions.pedro.PedroComponent.follower;
//
//import com.bylazar.configurables.annotations.IgnoreConfigurable;
//import com.bylazar.telemetry.TelemetryManager;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierCurve;
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.Path;
//import com.pedropathing.util.Timer;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
//import org.firstinspires.ftc.teamcode.PedroPathing.Tuning;
//
//import java.util.ArrayList;
//
//import dev.nextftc.extensions.pedro.FollowPath;
//import dev.nextftc.extensions.pedro.PedroComponent;
//import dev.nextftc.ftc.NextFTCOpMode;
//
//@Autonomous(name = "Forward Velocity Testing", group = "Auton")
//public class ForwardVelocityTesting extends NextFTCOpMode{
//    private final ArrayList<Double> velocities = new ArrayList<>();
//    public static double DISTANCE = 48;
//    public static double RECORD_NUMBER = 10;
//
//    @IgnoreConfigurable
//    static TelemetryManager telemetryM;
//
//    private boolean end;
//
//    @Override
//    public void init() {}
//
//    /** This initializes the drive motors as well as the cache of velocities and the Panels telemetry. */
//    @Override
//    public void init_loop() {
//        telemetryM.debug("The robot will run at 1 power until it reaches " + DISTANCE + " inches forward.");
//        telemetryM.debug("Make sure you have enough room, since the robot has inertia after cutting power.");
//        telemetryM.debug("After running the distance, the robot will cut power from the drivetrain and display the forward velocity.");
//        telemetryM.debug("Press B on game pad 1 to stop.");
//        telemetryM.debug("pose", Tuning.follower.getPose());
//        telemetryM.update(telemetry);
//
//        Tuning.follower.update();
//        drawCurrent();
//    }
//
//    /** This starts the OpMode by setting the drive motors to run forward at full power. */
//    @Override
//    public void start() {
//        for (int i = 0; i < RECORD_NUMBER; i++) {
//            velocities.add(0.0);
//        }
//        Tuning.follower.startTeleopDrive(true);
//        Tuning.follower.update();
//        end = false;
//    }
//
//    /**
//     * This runs the OpMode. At any point during the running of the OpMode, pressing B on
//     * game pad 1 will stop the OpMode. This continuously records the RECORD_NUMBER most recent
//     * velocities, and when the robot has run forward enough, these last velocities recorded are
//     * averaged and printed.
//     */
//    @Override
//    public void loop() {
//        if (gamepad1
//
//
//                .bWasPressed()) {
//            stopRobot();
//            requestOpModeStop();
//        }
//
//        Tuning.follower.update();
//        drawCurrentAndHistory();
//
//
//        if (!end) {
//            if (Math.abs(Tuning.follower.getPose().getX()) > DISTANCE) {
//                end = true;
//                stopRobot();
//            } else {
//                Tuning.follower.setTeleOpDrive(1,0,0,true);
//                //double currentVelocity = Math.abs(follower.getVelocity().getXComponent());
//                double currentVelocity = Math.abs(Tuning.follower.poseTracker.getLocalizer().getVelocity().getX());
//                velocities.add(currentVelocity);
//                velocities.remove(0);
//            }
//        } else {
//            stopRobot();
//            double average = 0;
//            for (double velocity : velocities) {
//                average += velocity;
//            }
//            average /= velocities.size();
//            telemetryM.debug("Forward Velocity: " + average);
//            telemetryM.debug("\n");
//            telemetryM.debug("Press A to set the Forward Velocity temporarily (while robot remains on).");
//
//            for (int i = 0; i < velocities.size(); i++) {
//                telemetry.addData(String.valueOf(i), velocities.get(i));
//            }
//
//            telemetryM.update(telemetry);
//            telemetry.update();
//
//            if (gamepad1.aWasPressed()) {
//                Tuning.follower.setXVelocity(average);
//                String message = "XMovement: " + average;
//                changes.add(message);
//            }
//        }
//    }
//}
