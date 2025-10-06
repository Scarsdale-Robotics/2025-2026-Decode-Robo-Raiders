package org.firstinspires.ftc.teamcode.Auton;

import com.pedropathing.geometry.BezierCurve;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.PedroPathing.Constants;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.extensions.pedro.*;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;

//Auton Naming Convention
//total slots = 4: __ __ __ __
//First slot = Name Type: Auton
//2nd slot = Classifier type (There can be multiple types on the same auto):
//1. Leaving it blank
//2. Wait (only for shooter type autons)
//3. Far (only for shooter and backup type autons)
//4. Close (only for shooter and backup type autons)
//3rd slot = Side type: Blue, Red
//4th slot = Auton type: Motif, Backup, Shooter
//Example Auton = AutonBlueCloseBackup, AutonRedWaitFarShooter ...
//Main Autons should be: Auton__WaitFarShooter & Auton__Motif

@Autonomous(name = "Auton Blue Motif", group = "Auton")
public class AutonBlueMotif extends NextFTCOpMode {
    public AutonBlueMotif() {
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

    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    /////////////////
    ////Positions////
    /////////////////
    // Positions the robot will be in during Auton
    // robotStartPath positions
    private final Pose startPose = new Pose(3, 136, Math.toRadians(0)); // Start Pose of our robot.
    private final Pose startPathControlP = new Pose (58, 108);
    private final Pose endBeforeMotifStartPose = new Pose(25, 100, Math.toRadians(270)); // End Pose of our robot.
    // MotifGPPIntake1 positions
    private final Pose motifGPPIntake1ControlP1 = new Pose(5, 60);
    private final Pose motifGPPIntake1ControlP2 = new Pose(47, 88);
    private final Pose motifGPPIntake1EndPos = new Pose(55, 94, Math.toRadians(30));
    // MotifGPPIntake2 intake preparation
    private final Pose motifGPPIntake2ControlP1 = new Pose(94, 93);
    private final Pose motifGPPIntake2MidPos = new Pose(37, 68, Math.toRadians(180));
    // MotifGPPIntake2 positions
    private final Pose motifGPPIntake2PushOutPurple = new Pose(37, 57.5);
    private final Pose motifGPPIntake2EndControlP1 = new Pose(45, 88);
    private final Pose motifGPPIntake2End = new Pose(37, 42, Math.toRadians(270));

    private final Pose motifGPPShooterEnd = new Pose(75, 90, Math.toRadians(180));

    // MotifPark
    private final Pose motifBlueParkControlP = new Pose(54, 14);
    private final Pose motifBluePark = new Pose(16, 5);




    /////////////
    ////Paths////
    /////////////
    // The different paths the robot will take in during Auton
    private Path robotStartPath;
    //Motif GPP Paths
    private PathChain motifGPP;
    private Path motifGPPIntake2;
    private Path motifGPPPark;
    //Motif GPG Paths
    private PathChain motifGPG;
    private Path motifGPGIntake1;
    private Path motifGPGIntake2;
    private Path motifGPGPark;
    //Motif PPG Paths
    private PathChain motifPPG;
    private Path motifPPGIntake1;
    private Path motifPPGIntake2;
    private Path motifPPGPark;

    ////////////////////
    ////Path Builder////
    ////////////////////
    // Builds the aforementioned paths with the initialized positions
    public void buildPaths() {
        /* This is the path before all the motif stuff. */
        robotStartPath = new Path(new BezierCurve(
                startPose,
                startPathControlP,
                endBeforeMotifStartPose));
        robotStartPath.setLinearHeadingInterpolation(startPose.getHeading(), endBeforeMotifStartPose.getHeading());

        // first Pattern motif, Green-Purple-Purple for now all paths are combined, but in future this will probably be separated.
        motifGPP = follower().pathBuilder()
                .addPath(new BezierCurve(
                        endBeforeMotifStartPose,
                        motifGPPIntake1ControlP1,
                        motifGPPIntake1ControlP2,
                        motifGPPIntake1EndPos))
                .setLinearHeadingInterpolation(endBeforeMotifStartPose.getHeading(), motifGPPIntake1EndPos.getHeading())

                .addPath(new BezierCurve(
                        motifGPPIntake1EndPos,
                        motifGPPIntake2ControlP1,
                        motifGPPIntake2MidPos))
                .setLinearHeadingInterpolation(motifGPPIntake1EndPos.getHeading(), motifGPPIntake2MidPos.getHeading())

                .addPath(new BezierLine(
                        motifGPPIntake2MidPos,
                        motifGPPIntake2PushOutPurple))

                .addPath(new BezierCurve(
                        motifGPPIntake2PushOutPurple,
                        motifGPPIntake2EndControlP1,
                        motifGPPIntake2End))
                .setLinearHeadingInterpolation(motifGPPIntake2MidPos.getHeading(), motifGPPIntake2End.getHeading())

                .addPath(new BezierLine(
                        motifGPPIntake2End,
                        motifGPPShooterEnd))
                .setLinearHeadingInterpolation(motifGPPIntake2End.getHeading(), motifGPPShooterEnd.getHeading())

                .addPath(new BezierCurve(
                        motifGPPShooterEnd,
                        motifBlueParkControlP,
                        motifBluePark))
                .build();
    }



    /////////////////////
    ////State Manager////
    /////////////////////
    // Controls which path the robot will take after finishing a specific path.
    public void autonomousPathUpdate() {
        switch (pathState) { //Although there may be no unique states useful for future Autons
            case 0:
                new FollowPath(robotStartPath);
                setPathState(-1); //Last Path
                break;
            case 1:
                // shouldn't do stuff
                break;
        }
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
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
