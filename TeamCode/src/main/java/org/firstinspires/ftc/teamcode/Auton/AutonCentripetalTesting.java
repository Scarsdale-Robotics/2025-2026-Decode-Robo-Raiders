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

@Autonomous(name = "Auton Centripetal Testing", group = "Auton")
public class AutonCentripetalTesting extends NextFTCOpMode{
    public AutonCentripetalTesting() {
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
                if (!follower().isBusy()) {
                    new FollowPath(robotPark);
                    setPathState(1); //Last Path
                    break;
                }
            case 1:
                if (!follower().isBusy()) {
                    new FollowPath(repeatedSideSteps);
                    setPathState(0);
                    break;
                }
        }
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @ Override
    public void runOpMode() {
        // These loop the movements of the robot, these must be called continuously in order to work
//        follower.update();
        autonomousPathUpdate();
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower().getPose().getX());
        telemetry.addData("y", follower().getPose().getY());
        telemetry.addData("heading", follower().getPose().getHeading());
        telemetry.update();
    }
    /** This method is called once at the init of the OpMode. **/
    @Override
    public void onInit() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
//        follower = Constants.createFollower(hardwareMap);

//        buildPaths();
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
