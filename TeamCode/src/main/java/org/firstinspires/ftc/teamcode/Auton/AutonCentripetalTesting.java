package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "Auton Centripetal + skib", group = "Auton")
public class AutonCentripetalTesting extends NextFTCOpMode{
//    public AutonCentripetalTesting() {
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
    private static double DISTANCE = 20;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    private final Path robotPark = new Path( new BezierCurve(
            new Pose(0,0),
            new Pose(Math.abs(DISTANCE),0),
            new Pose(Math.abs(DISTANCE),DISTANCE)));

    private final Path repeatedSideSteps = new Path(
            new BezierCurve(
                    new Pose(Math.abs(DISTANCE),DISTANCE),
                    new Pose(Math.abs(DISTANCE),0),
                    new Pose(0,0))
    );

    /////////////
    ////Paths////
    /////////////
    // The different paths the robot will take in during Auton

    ////////////////////
    ////Path Builder////
    ////////////////////
    // Builds the aforementioned paths with the initialized positions



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
//        waitForStart();
//        follower.update();
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
        follower = Constants.createFollower(hardwareMap);
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

//        buildPaths();
        follower.setStartingPose(new Pose(0,0));
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
