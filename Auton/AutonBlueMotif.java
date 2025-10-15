package org.firstinspires.ftc.teamcode.Auton;

import static org.firstinspires.ftc.teamcode.utils.QuarticMaxNonnegRoot.maxNonNegativeRoot;

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
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.outtake.TurretSubsystem;

import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.utility.InstantCommand;
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

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.outtake.TurretSubsystem;



//Auton Naming Convention
//total slots = 4: __ __ __ __
//First slot = Name Type: Auton
//2nd slot = Side type: Blue, Red
//3rd slot = Classifier type (There can be multiple types on the same auto):
//1. Leaving it blank
//2. Wait (only for shooter type autons)
//3. Far (only for shooter and backup type autons)
//4. Close (only for shooter and backup type autons)
//4th slot = Auton type: Motif, Backup, Shooter
//Example Auton = AutonBlueCloseBackup, AutonRedWaitFarShooter ...
//Main Autons should be: Auton__WaitFarShooter & Auton__Motif

/*
current Motif IDs
Motif GPP: 21
Motif PGP: 22
Motif PPG: 23
 */
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
    //////////////////////////
    ////METHODS(TEMPORARY)////
    //////////////////////////
    private void intakeOn() {
        //does absolutely nothing for now
    }
    private void intakeOff() {
        //does absolutely nothing for now
    }

    private Timer pathTimer, actionTimer, opmodeTimer;
    private int pathState;

    /////////////////
    ////Positions////
    /////////////////
    // Positions the robot will be in during Auton
    // robotStartPath positions
    private final Pose startPose = new Pose(33, 136, Math.toRadians(0)); // Start Pose of our robot.
    private final Pose startPathControlP = new Pose (58, 108);
    private final Pose endBeforeMotifStartPose = new Pose(25, 100, Math.toRadians(270)); // End Pose of our robot.
    ///MOTIF GPP///
    // MotifGPPIntake1
    private final Pose motifGPPIntake1IntakedControlP = new Pose(73, 87);
    private final Pose motifGPPIntake1Intaked = new Pose(20, 81, Math.toRadians(180));

    private final Pose motifGPPIntake1ToShoot = new Pose(58, 94);
    //MotifGPPIntake2 (also where it shoots the 2nd time)
    private final Pose motifGPPIntake2Purp1ControlP = new Pose(95, 134);
    private final Pose motifGPPIntake2Purp1Intaked = new Pose(37, 56);

    private final Pose motifGPPIntake2RestOTPControlP = new Pose(60, 90);
    private final Pose motifGPPIntake2RestOfThemPrep = new Pose(18.536, 69.658, Math.toRadians(-70));

    private final Pose motifGPPIntake2Intaked = new Pose(19.364, 68.7);
    private final Pose motifGPPIntake2Intaked2 = new Pose(29, 67.9);

    ///MOTIF PGP///
    // MotifPGPIntake1
    private final Pose motifPGPIntake1PrepControlP = new Pose(43, 98);
    private final Pose motifPGPIntake1Prep = new Pose(37, 84, Math.toRadians(180));

    private final Pose motifPGPIntake1IntakedControl = new Pose(32, 112);
    private final Pose motifPGPIntake1Intaked = new Pose(17.45, 93, Math.toRadians(-70));

    private final Pose motifPGPIntake1Intaked2 = new Pose(20.9, 90.9);

    private final Pose motifPGPIntake1FinalPos = new Pose(56, 94);

    // MotifPGPIntake2
    private final Pose motifPGPIntake2PrepControlP = new Pose(101, 120);
    private final Pose motifPGPIntake2Prep = new Pose(37, 60, Math.toRadians(180));

    private final Pose motifPGPIntake2IntakedControlP = new Pose(1.5, 58);


    ///MOTIF PPG///
    // MotifPPGIntake1 prep
    private final Pose motifPPGIntake1StartPos = new Pose (18.2, 92.6, Math.toRadians(-65));
    private final Pose motifPPGIntake1MidPos = new Pose (18.2, 89.5);
    // MotifPPGIntake 1 position
    private final Pose motifPPGIntake1EndPos = new Pose(59, 94, Math.toRadians(0));
    // MotifPPGIntake2 intake preparation
    private final Pose motifPPGIntake2ControlP1 = new Pose(94, 93);
    private final Pose motifPPGIntake2MidPos = new Pose(37, 68, Math.toRadians(180));
    // MotifPPGIntake2 positions
    private final Pose motifPPGIntake2PushOutPurple = new Pose(37, 57.5);
    private final Pose motifPPGIntake2EndControlP1 = new Pose(45, 88);
    private final Pose motifPPGIntake2End = new Pose(37, 42, Math.toRadians(270));


    ///Motif Park///
    private final Pose motifsShooterEnd = new Pose(75, 90, Math.toRadians(180));

    private final Pose motifBlueParkControlP = new Pose(54, 14);
    private final Pose motifBluePark = new Pose(16, 5);




    /////////////
    ////Paths////
    /////////////
    // The different paths the robot will take in during Auton
    private Path robotStartPath;
    //Motif GPP Paths
    private PathChain motifGPPIntake1;
    private PathChain motifGPPIntake2;
    private Path motifGPPLastShot;
    //Motif PGP Paths
    private PathChain motifPGPIntake1;
    private PathChain motifPGPIntake2;
    //Motif PPG Paths
    private PathChain motifPPGIntake1Prep;
    private Path motifPPGIntake1;
    private PathChain motifPPGIntake2;
    private Path motifPPGLastShot;

    private Path motifPark; //wants to put end pos in the folder: org.firstinspires.ftc.teamcode

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

        // 1st Pattern motif, Green-Purple-Purple
        motifGPPIntake1 = follower().pathBuilder()
                .addPath(new BezierCurve(
                        endBeforeMotifStartPose,
                        motifGPPIntake1IntakedControlP,
                        motifGPPIntake1Intaked))
                .setLinearHeadingInterpolation(endBeforeMotifStartPose.getHeading(), motifGPPIntake1Intaked.getHeading())

                .addPath(new BezierLine(
                        motifGPPIntake1Intaked,
                        motifGPPIntake1ToShoot))
                .build();

        motifGPPIntake2 = follower().pathBuilder()
                .addPath(new BezierCurve(
                        motifGPPIntake1ToShoot,
                        motifGPPIntake2Purp1ControlP,
                        motifGPPIntake2Purp1Intaked))
//                .setLinearHeadingInterpolation(endBeforeMotifStartPose.getHeading(), motifGPPIntake1Intaked.getHeading())

                .addPath(new BezierCurve(
                        motifGPPIntake2Purp1Intaked,
                        motifGPPIntake2RestOTPControlP,
                        motifGPPIntake2RestOfThemPrep))
                .setLinearHeadingInterpolation(motifGPPIntake1Intaked.getHeading(), motifGPPIntake2RestOfThemPrep.getHeading())

                .addPath(new BezierLine(
                        motifGPPIntake2RestOfThemPrep,
                        motifGPPIntake2Intaked))
                .addPath (new BezierLine(
                        motifGPPIntake2Intaked,
                        motifGPPIntake2Intaked2))
                .build();

        motifGPPLastShot = new Path(new BezierLine(
                motifGPPIntake2Intaked,
                motifsShooterEnd));
        motifPPGLastShot.setLinearHeadingInterpolation(motifGPPIntake2RestOfThemPrep.getHeading(), motifsShooterEnd.getHeading());

        // 2nd Pattern motif, Purple-Green-Purple
        motifPGPIntake1 = follower().pathBuilder()
                .addPath(new BezierCurve(
                        endBeforeMotifStartPose,
                        motifPGPIntake1PrepControlP,
                        motifPGPIntake1Prep))
                .setLinearHeadingInterpolation(endBeforeMotifStartPose.getHeading(), motifPGPIntake1Prep.getHeading())

                .addPath(new BezierCurve(
                        motifPGPIntake1Prep,
                        motifPGPIntake1IntakedControl,
                        motifPGPIntake1Intaked))
                .setLinearHeadingInterpolation(motifPGPIntake1Prep.getHeading(), motifPGPIntake1Intaked.getHeading())

                .addPath (new BezierLine(
                        motifPGPIntake1Intaked,
                        motifPGPIntake1Intaked2))

                .addPath (new BezierLine(
                        motifPGPIntake1Intaked2,
                        motifPGPIntake1FinalPos))
                .build();

        motifPGPIntake2 = follower().pathBuilder()
                .addPath(new BezierCurve(
                        motifPGPIntake1FinalPos,
                        motifPGPIntake2PrepControlP,
                        motifPGPIntake2Prep))
                .setLinearHeadingInterpolation(motifPGPIntake1Intaked.getHeading(), motifPGPIntake2Prep.getHeading())

                .addPath(new BezierCurve(
                        motifPGPIntake2Prep,
                        motifPGPIntake2IntakedControlP,
                        motifsShooterEnd))
                .build();

        // 3rd Pattern motif, Purple-Purple-Green for now all paths are combined, but in future this will probably be separated.
        motifPPGIntake1Prep = follower().pathBuilder()
                .addPath(new BezierLine(
                        endBeforeMotifStartPose,
                        motifPPGIntake1StartPos))
                .setLinearHeadingInterpolation(endBeforeMotifStartPose.getHeading(), motifPPGIntake1StartPos.getHeading())

                .addPath(new BezierLine(
                        motifPPGIntake1StartPos,
                        motifPPGIntake1MidPos))
                .build();

        motifPPGIntake1 = new Path(new BezierLine(
                motifPPGIntake1MidPos,
                motifPPGIntake1EndPos));
        robotStartPath.setLinearHeadingInterpolation(motifPPGIntake1StartPos.getHeading(), motifPPGIntake1EndPos.getHeading());

        motifPPGIntake2 = follower().pathBuilder()
                .addPath(new BezierCurve(
                        motifPPGIntake1EndPos,
                        motifPPGIntake2ControlP1,
                        motifPPGIntake2MidPos))
                .setLinearHeadingInterpolation(motifPPGIntake1EndPos.getHeading(), motifPPGIntake2MidPos.getHeading())

                .addPath(new BezierLine(
                        motifPPGIntake2MidPos,
                        motifPPGIntake2PushOutPurple))

                .addPath(new BezierCurve(
                        motifPPGIntake2PushOutPurple,
                        motifPPGIntake2EndControlP1,
                        motifPPGIntake2End))
                .setLinearHeadingInterpolation(motifPPGIntake2MidPos.getHeading(), motifPPGIntake2End.getHeading())
                .build();

        motifPPGLastShot = new Path(new BezierLine(
                        motifPPGIntake2End,
                        motifsShooterEnd));
        motifPPGLastShot.setLinearHeadingInterpolation(motifPPGIntake2End.getHeading(), motifsShooterEnd.getHeading());


        //last path to follow, the park path
        motifPark = new Path(new BezierCurve(
                motifsShooterEnd,
                motifBlueParkControlP,
                motifBluePark));
    }



    /////////////////////
    ////State Manager////
    /////////////////////
    // Controls which path the robot will take after finishing a specific path.
    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                CommandManager.INSTANCE.scheduleCommand(OuttakeSubsystem.INSTANCE.shootWhenReady);
                new FollowPath(robotStartPath);
                setPathState(3); //for now 3
                break;
            case 1: //Motif GPP starts
                if (!follower().isBusy()) {
                    CommandManager.INSTANCE.scheduleCommand(IntakeSubsystem.INSTANCE.setPower(1)); //(TURNS ON INTAKE) temporary value
                    new FollowPath(motifGPPIntake1);
                    if (follower().atPose(endBeforeMotifStartPose, 1, 1)) {
                        setPathState(7);
                    }
                }
                break;
            case 2: //Motif PGP starts
                if (!follower().isBusy()) {
                    CommandManager.INSTANCE.scheduleCommand(IntakeSubsystem.INSTANCE.setPower(1)); //(TURNS ON INTAKE) temporary value
                    new FollowPath(motifPGPIntake1);
                    if (follower().atPose(endBeforeMotifStartPose, 1, 1)) {
                        setPathState(9);
                    }
                }
                break;
            case 3: //Motif PPG starts (INTAKE1 PREP)
                if (!follower().isBusy()) {
                    CommandManager.INSTANCE.scheduleCommand(IntakeSubsystem.INSTANCE.setPower(1)); //(TURNS ON INTAKE) temporary value
                    new FollowPath(motifPPGIntake1Prep);
                    if (follower().atPose(endBeforeMotifStartPose, 1, 1)) {
                        setPathState(4);
                    }
                }
                break;
            case 4: //Motif PPG sequence 1 (INTAKE1 LOAD)
                if (!follower().isBusy()) {
                    new FollowPath(motifPPGIntake1);
                    if (follower().atPose(motifPPGIntake1EndPos, 1, 1)) {
                        CommandManager.INSTANCE.scheduleCommand(IntakeSubsystem.INSTANCE.setPower(0)); //(TURNS OFF INTAKE) temporary value
                    }
                    if (follower().atPose(motifPPGIntake1EndPos, 1, 1)) {
                        setPathState(5);
                    }
                }
                break;
            case 5: //Motif PPG sequence 2 ( (INTAKE2 LOAD)
                if (!follower().isBusy()) {
                    CommandManager.INSTANCE.scheduleCommand(OuttakeSubsystem.INSTANCE.shootWhenReady);
                    if (follower().atPose(motifPPGIntake2MidPos, 1, 1)) {
                        CommandManager.INSTANCE.scheduleCommand(IntakeSubsystem.INSTANCE.setPower(1)); //(TURNS ON INTAKE) temporary value
                    }
                    new FollowPath(motifPPGIntake2);
                    if (follower().atPose(motifPPGIntake2End, 1, 1)) {
                        CommandManager.INSTANCE.scheduleCommand(IntakeSubsystem.INSTANCE.setPower(0)); //(TURNS OFF INTAKE) temporary value
                    }
                    if (follower().atPose(motifPPGIntake2End, 1, 1)) {
                        setPathState(6);
                    }
                }
                break;
            case 6: //Motif PPG sequence 3 (GO TO LAST SHOOT POS)
                if (!follower().isBusy()) {
                    new FollowPath(motifPPGLastShot);
                    setPathState(999);
                }
                break;
            case 7: //Motif GPP sequence 1 (INTAKE1 LOAD)
                if (!follower().isBusy()) {
                    new FollowPath(motifGPPIntake2);
                    if (follower().atPose(motifGPPIntake1ToShoot, 1, 1)) {
                        CommandManager.INSTANCE.scheduleCommand(IntakeSubsystem.INSTANCE.setPower(0)); //(TURNS OFF INTAKE) temporary value
                        CommandManager.INSTANCE.scheduleCommand(OuttakeSubsystem.INSTANCE.shootWhenReady);
                    }
                    if (follower().atPose(motifGPPIntake2Purp1Intaked, 5, 5)) {
                        CommandManager.INSTANCE.scheduleCommand(IntakeSubsystem.INSTANCE.setPower(1)); //(TURNS ON INTAKE) temporary value
                    }
                    if (follower().atPose(motifGPPIntake2Intaked2, 1, 1)) {
                        setPathState(8);
                    }
                }
                break;
            case 8: //Motif GPP sequence 2
                if (!follower().isBusy()) {
                    new FollowPath(motifGPPLastShot);
                    setPathState(999);
                }
                break;
            case 9: //Motif PGP sequence 1 (INTAKE1 LOAD)
                if (!follower().isBusy()) {
                    new FollowPath(motifPGPIntake1);
                    if (follower().atPose(motifPGPIntake1FinalPos, 1, 1)) {
                        CommandManager.INSTANCE.scheduleCommand(IntakeSubsystem.INSTANCE.setPower(0)); //(TURNS OFF INTAKE) temporary value
                    }
                    if (follower().atPose(motifPPGIntake1EndPos, 1, 1)) {
                        setPathState(10);
                    }
                }
                break;
            case 10: //Motif PGP sequence 2 (INTAKE2 LOAD)
                if (!follower().isBusy()) {
                    new FollowPath(motifPGPIntake2);
                    CommandManager.INSTANCE.scheduleCommand(OuttakeSubsystem.INSTANCE.shootWhenReady);
                    if (follower().atPose(motifPGPIntake2Prep, 5, 5)) {
                        CommandManager.INSTANCE.scheduleCommand(IntakeSubsystem.INSTANCE.setPower(1)); //(TURNS ON INTAKE) temporary value
                    }
                    if (follower().atPose(motifsShooterEnd, 1, 1)) {
                        CommandManager.INSTANCE.scheduleCommand(IntakeSubsystem.INSTANCE.setPower(0)); //(TURNS OFF INTAKE) temporary value
                    }
                    if (follower().atPose(motifsShooterEnd, 1, 1)) {
                        setPathState(999);
                    }
                }
                break;
            case 999: //Motif park for all, current case # is temporary
                if (!follower().isBusy()) {
                    CommandManager.INSTANCE.scheduleCommand(OuttakeSubsystem.INSTANCE.shootWhenReady);
                    if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                        new FollowPath(motifPark);
                    }
                    setPathState(-1); //last path
                }
                break;

        }
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void runOpMode() {
        // These loop the movements of the robot, these must be called continuously in order to work
//        follower.update();
        autonomousPathUpdate();
//        if (pathTimer.getElapsedTimeSeconds() > 1) {
//            CommandManager.INSTANCE.scheduleCommand(TurretSubsystem.INSTANCE.autoAim(telemetry));
//        }
        CommandManager.INSTANCE.scheduleCommand(TurretSubsystem.INSTANCE.autoAim(telemetry));
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
        follower().setStartingPose(startPose);
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
