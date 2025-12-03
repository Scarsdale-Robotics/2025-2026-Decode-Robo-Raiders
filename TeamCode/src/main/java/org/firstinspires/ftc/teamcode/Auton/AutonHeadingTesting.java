package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "Auton Heading Testing", group = "Auton")
public class AutonHeadingTesting extends NextFTCOpMode {
//    public AutonHeadingTesting() {
//        addComponents(
//                new PedroComponent(Constants::createFollower)
//        );
//    }
    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    private Follower follower;
    private static double DISTANCE = 40;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    /////////////////
    ////Positions////
    /////////////////
    // Positions the robot will be in during Auton
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0)); // Start Pose of our robot.
    private final Pose endPose = new Pose(DISTANCE, 0, Math.toRadians(90)); // End Pose of our robot.

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



    /////////////////////
    ////State Manager////
    /////////////////////
    // Controls which path the robot will take after finishing a specific path.
    public void autonomousPathUpdate() {
        switch (pathState) { //Although there may be no unique states useful for future Autons
            case 0:
                if (!follower.isBusy()) {
                    follower.followPath(robotPark);
                    setPathState(1); //Last Path
                    break;
                }
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(repeatedSideSteps);
                    setPathState(0);
                    break;
                }
        }
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @ Override
    public void onUpdate() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
    /** This method is called once at the init of the OpMode. **/
    @Override
    public void onInit() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setStartingPose(startPose);
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
