package org.firstinspires.ftc.teamcode.Auton;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.PedroPathing.Constants;

import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "Auton Translational Testing", group = "Auton")
public class AutonTranslationalTesting extends NextFTCOpMode{
    public AutonTranslationalTesting() {
        addComponents(
                new PedroComponent(Constants::createFollower)
        );
    }
    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    private static double DISTANCE = 40;
    private boolean forward = true;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    /////////////////
    ////Positions////
    /////////////////
    // Positions the robot will be in during Auton
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0)); // Start Pose of our robot.
    private final Pose endPose = new Pose(DISTANCE, 0, Math.toRadians(0)); // End Pose of our robot.

    /////////////
    ////Paths////
    /////////////
    // The different paths the robot will take in during Auton
    private Path robotPark;
    private Path repeatedSideSteps;

    ////////////////////
    ////Path Builder////
    ////////////////////
    // Builds the aforementioned paths with the initialized positions
    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        robotPark = new Path(new BezierLine(startPose, endPose));
        robotPark.setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading());

        repeatedSideSteps = new Path(new BezierLine(endPose, startPose));
        repeatedSideSteps.setLinearHeadingInterpolation(endPose.getHeading(), startPose.getHeading());
    }


    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @ Override
    public void runOpMode() {
        // These loop the movements of the robot, these must be called continuously in order to work
//        follower.update();
        if (!follower().isBusy()) {
            if (forward) {
                forward = false;
                new FollowPath(repeatedSideSteps);
            } else {
                forward = true;
                new FollowPath(robotPark);
            }
            // Feedback to Driver Hub for debugging
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower().getPose().getX());
            telemetry.addData("y", follower().getPose().getY());
            telemetry.addData("heading", follower().getPose().getHeading());
            telemetry.update();
        }
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void onInit() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
//        follower = Constants.createFollower(hardwareMap);

        buildPaths();
//        follower.setStartingPose(startPose);
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void onWaitForStart() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void onStartButtonPressed() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }
    /** We do not use this because everything should automatically disable **/
    @Override
    public void onStop() {}

}
