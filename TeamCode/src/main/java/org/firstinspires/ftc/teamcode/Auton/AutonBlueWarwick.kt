package org.firstinspires.ftc.teamcode.Auton
import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.ftc.NextFTCOpMode
//import org.firstinspires.ftc.teamcode.opmodes.testing.archive.TeleOpInProg.Companion.goalX
//import org.firstinspires.ftc.teamcode.opmodes.testing.archive.TeleOpInProg.Companion.goalY
import org.firstinspires.ftc.teamcode.opmodes.testing.archive.TeleOpInProg.Companion.m
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.MagazineServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.MagazineServoSubsystem.forward
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.MagblockServoSubsystem.close
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.MagblockServoSubsystem.open
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.PusherServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.PusherServoSubsystem.`in`
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.PusherServoSubsystem.out
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem
import org.firstinspires.ftc.teamcode.utils.Lefile.filePath

import java.io.File

import kotlin.math.hypot
//
////Auton Naming Convention
////total slots = 4: __ __ __ __
////First slot = Name Type: Auton
////2nd slot = Side type: Blue, Red
////3rd slot = Classifier type (There can be multiple types on the same auto):
////1. Leaving it blank
////2. Wait (only for shooter type autons)
////3. Far (only for shooter and backup type autons)
////4. Close (only for shooter and backup type autons)
////4th slot = Auton type: Motif, Backup, Shooter, Artifact (is just Motif but does not care about motifs)
////Example Auton = AutonBlueCloseBackup, AutonRedWaitFarShooter ...
////Main Autons should be: Auton__WaitFarShooter & Auton__Motif
@Autonomous(name = "WARICK Blue Auton", group = "Auton")
@Configurable
class AutonBlueWarwick : NextFTCOpMode() {
    private var pathTimer: Timer? = null
    var actionTimer: Timer? = null;
    var opmodeTimer: Timer? = null;

    val delay3rdBall: Double = 2.5
    val afterPushDelay: Double = 0.2

    val distanceGoalX = 12
    val distanceGoalY = 132
    var directionGoalX = 4.0;
    var directionGoalY = 144.0-4.0;

    init {
        addComponents(
            SubsystemComponent(
                LowerSubsystem,
                OuttakeSubsystem,
                IntakeSubsystem
            )
        );

        actionTimer = Timer()
        pathTimer = Timer()
        opmodeTimer = Timer()
        opmodeTimer!!.resetTimer()
    }

    enum class AutonPath {
        RobotShoot1, //the starting path of the robot
        RobotIntake1,
        RobotShoot2,
        RobotIntake2,
        RobotShoot3,
        RobotBallRelease,
        EndAuton
    }

    private var pathState: AutonPath = AutonPath.RobotShoot1

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     */
    fun setPathState(pState: AutonPath) {
        pathState = pState
        pathTimer?.resetTimer()
    }

    var follower: Follower? = null

    /////////////////
    ////Positions////
    /////////////////
    // Positions the robot will be in during Auton
    // robot positions
    private val startPose = Pose(32.0, 137.0, Math.toRadians(180.0)) // Start Pose of our robot.

    private val intake1stLinePos = Pose(11.0, 61.5)
    private val intake1ControlPointPos = Pose(73.0, 52.0)

    private val intake2ndLinePos = Pose(16.5, 85.5)
    private val intake2ControlPointPos = Pose(45.0, 87.0)
    private val intake2FirstBallPos = Pose(27.0, intake2ndLinePos.y)

//    private val intake3rdLinePos = Pose(8.0, 36.0)
//    private val intake3ControlPointPos = Pose(77.0, 33.0)
//    private val intake3FirstBallPos = Pose(27.0, intake3rdLinePos.y)

    private val ballReleasePose = Pose(15.5, 66.0, Math.toRadians(90.0))

    private val shootingPose = Pose(58.0, 80.0, Math.toRadians(180.0)) // The shooter position for everything
    private val endPose = Pose(48.0, 80.0) // End Pose of our robot

    companion object {
        private val toleranceIntakeMagSeq = 5.0

        private var magBallHitDelay = 1.0  // pessimistic time to hit magblock
        private var magBallEnterDelay = magBallHitDelay + 2.5  // time to pass magblock

        private var intakeMaxPower = 1.0
        private var shootReturnPower = 1.0
        private var delayAfterIntake = 0.42

        private var delayOut = 0.5;

        private var intakeMagblockDelay = 0.2;

        private var intakeEndPosTolerance = 2.0;
        private var shootingPoseTolerance = 3.0;
    }

    /////////////
    ////Paths////
    /////////////
    // The different paths the robot will take in during Auton
    private var robotShootPreload: PathChain? = null

    private var robotIntake1: PathChain? = null
    private var robotGoToShoot1: PathChain? = null

    private var robotIntake2: PathChain? = null
    private var robotGoToShoot2: PathChain? = null

    private var robotWarwickEssential: PathChain? = null
//
//    private var robotIntake3: PathChain? = null
//    private var robotGoToShoot3: PathChain? = null

    private var robotGoToShoot4: PathChain? = null


    //wants to put end pos in the folder: org.firstinspires.ftc.teamcode


    ////////////////////
    ////Path Builder////
    ////////////////////
    // Builds the aforementioned paths with the initialized positions
    fun buildPaths() {
        /* This is the path before all the motif stuff. */
        robotShootPreload = follower!!.pathBuilder()
            .addPath(
                BezierLine(
                    startPose,
                    shootingPose
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()
//        robotShootPreload!!.setLinearHeadingInterpolation(
//            startPose.heading,
//            shootingPose.heading
//        )

        // 1st Intake
        robotIntake1 = follower!!.pathBuilder()
            .addPath(
                BezierCurve(
                    shootingPose,
                    intake1ControlPointPos,
                    intake1stLinePos
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()
        //1st Go Shoot
        robotGoToShoot1 = follower!!.pathBuilder()
            .addPath(
                BezierLine(
                    intake1stLinePos,
                    shootingPose
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()

        // 2nd Intake
        robotIntake2 = follower!!.pathBuilder()
            .addPath(
                BezierCurve(
                    shootingPose,
                    intake2ControlPointPos,
                    intake2ndLinePos
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()
        //2nd Go Shoot
        robotGoToShoot2 = follower!!.pathBuilder()
            .addPath(
                BezierLine(
                    intake2ndLinePos,
                    shootingPose
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()

        //WARWICK
        robotWarwickEssential = follower!!.pathBuilder()
            .addPath(
                BezierLine(
                    intake2ndLinePos,
                    shootingPose
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(90.0))
            .build()

        //4th Go Shoot
        robotGoToShoot4 = follower!!.pathBuilder()
            .addPath(
                BezierLine(
                    ballReleasePose,
                    endPose
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()
    }
    var pathF1: Boolean = true
    var pathF2: Boolean = true
    var pathF3: Boolean = true
    var pathF4: Boolean = true
    var pathF5: Boolean = true

    var pusherSetUp1: Boolean = true
    var pusherSetUp2: Boolean = true
    var pusherSetUp3: Boolean = true
    var pusherSetUp4: Boolean = true
    var pusherSetUp5: Boolean = true

    var magSeqReady1: Boolean = true
    var magSeqReady2: Boolean = true
    var magSeqReady3: Boolean = true
    var magSeqReady4: Boolean = true
    var magSeqReady5: Boolean = true

    var intakeReached1: Boolean = true
    var intakeReached2: Boolean = true
    var intakeReached3: Boolean = true
    var intakeReached4: Boolean = true

    var intakeDone1: Boolean = true
    var intakeDone2: Boolean = true
    var intakeDone3: Boolean = true
    var intakeDone4: Boolean = true

    /////////////////////
    ////State Manager////
    /////////////////////
    // Controls which path the robot will take after finishing a specific path.
    fun autonomousPathUpdate() {
        when (pathState) {
            AutonPath.RobotShoot1 -> {
                follower!!.setMaxPower(1.0)
                forward.schedule()
                if (pathF1) {
                    follower!!.followPath(robotShootPreload!!)
                    close.schedule()
                    pathF1 = false
                }
                if (follower!!.atPose(shootingPose, shootingPoseTolerance, shootingPoseTolerance)) { //Shooting stuff
                    intake.schedule()
                    if (pusherSetUp1) {
                        pusherSetUp1 = false
                        actionTimer!!.resetTimer()
                    }
                    PanelsTelemetry.telemetry.addData("actionTimer", actionTimer!!.elapsedTimeSeconds)
                    PanelsTelemetry.telemetry.addData("pathTimer", pathTimer!!.elapsedTimeSeconds)
                    if (actionTimer!!.elapsedTimeSeconds >= 1.0){
                        open.schedule()
                    }
                    if (actionTimer!!.elapsedTimeSeconds >= delay3rdBall + 1.0) {
                        `in`.schedule()
                    }
                    if (actionTimer!!.elapsedTimeSeconds >= delay3rdBall + afterPushDelay + 1.0) {
                        setPathState(AutonPath.RobotIntake1)
                    }
                }
            }

            AutonPath.RobotIntake1 -> if (!follower!!.isBusy) {
                follower!!.setMaxPower(intakeMaxPower)
                if (pathTimer!!.elapsedTimeSeconds > delayOut) {
                    out.schedule()
                }
                if (intakeReached1) {
                    follower!!.followPath(robotIntake1!!)
                    intakeReached1 = false
                }
                if(follower!!.atPose(intake1stLinePos,
                        intakeEndPosTolerance,
                        intakeEndPosTolerance
                    )) {
                    if (intakeDone1) {
                        actionTimer!!.resetTimer()
                        intakeDone1 = false
                    }
                }
                if (!intakeDone1) {
                    if (actionTimer!!.elapsedTimeSeconds >= intakeMagblockDelay) {
                        close.schedule()
                    }
                    if (actionTimer!!.elapsedTimeSeconds >= delayAfterIntake + 0.1) {
                        setPathState(AutonPath.RobotShoot2)
                    }
                }
            }

            AutonPath.RobotShoot2 -> if (!follower!!.isBusy) {
                follower!!.setMaxPower(shootReturnPower)
                if (pathF2) {
                    follower!!.followPath(robotGoToShoot1!!)
                    pathF2 = false
                }
                if (follower!!.atPose(shootingPose,
                        shootingPoseTolerance,
                        shootingPoseTolerance
                    )) { //Shooting stuff
                    open.schedule()
                    intake.schedule()
                    if (pusherSetUp2) {
                        pusherSetUp2 = false
                        actionTimer!!.resetTimer()
                    }
                    PanelsTelemetry.telemetry.addData("actionTimer", actionTimer!!.elapsedTimeSeconds)
                    PanelsTelemetry.telemetry.addData("pathTimer", pathTimer!!.elapsedTimeSeconds)
                    if (actionTimer!!.elapsedTimeSeconds >= delay3rdBall) {
                        `in`.schedule()
                    }
                    if (actionTimer!!.elapsedTimeSeconds >= delay3rdBall + afterPushDelay) {
                        setPathState(AutonPath.RobotIntake2)
                    }
                }
            }

            AutonPath.RobotIntake2 -> if (!follower!!.isBusy) {
                follower!!.setMaxPower(intakeMaxPower)
                if (pathTimer!!.elapsedTimeSeconds > delayOut) {
                    out.schedule()
                }
                if (intakeReached2) {
                    follower!!.followPath(robotIntake2!!)
                    intakeReached2 = false
                }
                if(follower!!.atPose(intake2ndLinePos,
                        intakeEndPosTolerance,
                        intakeEndPosTolerance
                    )) {
                    if (intakeDone2) {
                        actionTimer!!.resetTimer()
                        intakeDone2 = false
                    }
                }
                if (!intakeDone2) {
                    if (actionTimer!!.elapsedTimeSeconds >= intakeMagblockDelay) {
                        close.schedule()
                    }
                    if (actionTimer!!.elapsedTimeSeconds >= delayAfterIntake) {
                        setPathState(AutonPath.RobotShoot3)
                    }
                }
            }

            AutonPath.RobotShoot3 -> if (!follower!!.isBusy) {
                follower!!.setMaxPower(shootReturnPower)
                if (follower!!.atPose(intake2FirstBallPos,
                        toleranceIntakeMagSeq,
                        toleranceIntakeMagSeq
                    )) {
                    if (magSeqReady2) {
                        magSeqReady2 = false
                        actionTimer!!.resetTimer()
                    }
                }
                if (pathF3) {
                    follower!!.followPath(robotGoToShoot2!!)
                    pathF3 = false
                }

                if (follower!!.atPose(shootingPose,
                        shootingPoseTolerance,
                        shootingPoseTolerance
                    )) { //Shooting stuff
                    open.schedule()
                    intake.schedule()
                    if (pusherSetUp3) {
                        pusherSetUp3 = false
                        actionTimer!!.resetTimer()
                    }
                    PanelsTelemetry.telemetry.addData("actionTimer", actionTimer!!.elapsedTimeSeconds)
                    PanelsTelemetry.telemetry.addData("pathTimer", pathTimer!!.elapsedTimeSeconds)
                    if (actionTimer!!.elapsedTimeSeconds >= delay3rdBall) {
                        `in`.schedule()
                    }
                    if (actionTimer!!.elapsedTimeSeconds >= delay3rdBall + afterPushDelay) {
                        setPathState(AutonPath.RobotBallRelease)
                    }
                }
            }

            AutonPath.RobotBallRelease -> if (!follower!!.isBusy) {
                follower!!.setMaxPower(0.5)
                if (intakeReached3) {
                    stop.schedule()
                    follower!!.followPath(robotWarwickEssential!!)
                    intakeReached3 = false
                }
                if(follower!!.atPose(ballReleasePose, intakeEndPosTolerance, intakeEndPosTolerance)) {
                    if (intakeDone2) {
                        actionTimer!!.resetTimer()
                        intakeDone2 = false
                    }
                }
                if (!intakeDone2) {
                    if (actionTimer!!.elapsedTimeSeconds >= 2) {
                        setPathState(AutonPath.EndAuton)
                    } else if (actionTimer!!.elapsedTimeSeconds >= intakeMagblockDelay) {
                        close.schedule()
                    }
                }
            }

//            AutonPath.RobotIntake3 -> if (!follower!!.isBusy) {
//                follower!!.setMaxPower(intakeMaxPower)
//                if (intakeReached3) {
//                    follower!!.followPath(robotIntake3!!)
//                    intakeReached3 = false
//                }
//                if(follower!!.atPose(intake3rdLinePos, intakeEndPosTolerance, intakeEndPosTolerance)) {
//                    if (intakeDone3) {
//                        actionTimer!!.resetTimer()
//                        intakeDone3 = false
//                        close.schedule()
//                    }
//                }
//                if (!intakeDone3) {
//                    if (actionTimer!!.elapsedTimeSeconds >= delayAfterIntake) {
//                        setPathState(AutonPath.RobotShoot4)
//                    } else if (actionTimer!!.elapsedTimeSeconds >= intakeMagblockDelay) {
//                        close.schedule()
//                    }
//                }
//            }
//
//            AutonPath.RobotShoot4 -> if (!follower!!.isBusy) { //MAY BE REACHED // WOULD MEAN 12 ARTIFACT AUTON //
//                follower!!.setMaxPower(shootReturnPower)
//                if (follower!!.atPose(intake3FirstBallPos, toleranceIntakeMagSeq, toleranceIntakeMagSeq)) {
//                    if (magSeqReady3) {
//                        magSeqReady3 = false
//                        actionTimer!!.resetTimer()
//                    }
//                }
//                if (pathF4) {
//                    follower!!.followPath(robotGoToShoot3!!)
//                    pathF4 = false
//                }
//                if (follower!!.atPose(shootingPose, shootingPoseTolerance, shootingPoseTolerance)) { //Shooting stuff
//                    open.schedule()
//                    intake.schedule()
//                    if (pusherSetUp4) {
//                        pusherSetUp4 = false
//                        actionTimer!!.resetTimer()
//                    }
//                    PanelsTelemetry.telemetry.addData("actionTimer", actionTimer!!.elapsedTimeSeconds)
//                    PanelsTelemetry.telemetry.addData("pathTimer", pathTimer!!.elapsedTimeSeconds)
//                    if (actionTimer!!.elapsedTimeSeconds >= delay3rdBall + afterPushDelay) {
//                        out.schedule()
//                        setPathState(AutonPath.EndAuton)
//                    } else if (actionTimer!!.elapsedTimeSeconds >= delay3rdBall) {
//                        `in`.schedule()
//                    }
//                }
//                if (opmodeTimer!!.elapsedTimeSeconds >= 29.5) {
//                    out.schedule()
//                    setPathState(AutonPath.EndAuton)
//                }
//            }
//
//            AutonPath.RobotIntake4 -> if (!follower!!.isBusy) {
//                follower!!.setMaxPower(intakeMaxPower)
//                if (intakeReached4) {
//                    follower!!.followPath(robotIntake4!!)
//                    intakeReached4 = false
//                }
//                if(follower!!.atPose(intake4thLinePos, 0.15, 0.15)) {
//                    if (intakeDone4) {
//                        actionTimer!!.resetTimer()
//                        intakeDone4 = false
//                    }
//                    if (actionTimer!!.elapsedTimeSeconds >= delayAfterIntake) {
//                        setPathState(AutonPath.RobotShoot5)
//                    } else if (actionTimer!!.elapsedTimeSeconds >= intakeMagblockDelay) {
//                        close.schedule()
//                    }
//                }
//            }
//
//            AutonPath.RobotShoot5 -> if (!follower!!.isBusy) { //CANNOT BE REACHED // WOULD MEAN 14 ARTIFACT AUTON //
//                follower!!.setMaxPower(shootReturnPower)
//                // BAD NEED SETUP
//                if (follower!!.atPose(intake4FirstBallPos, toleranceIntakeMagSeq, toleranceIntakeMagSeq)) {
//                    if (magSeqReady4) {
//                        magSeqReady4 = false
//                        actionTimer!!.resetTimer()
//                    }
//                }
//                if (pathF5) {
//                    follower!!.followPath(robotGoToShoot4!!)
//                    pathF5 = false
//                }
//                if (follower!!.atPose(shootingPose, shootingPoseTolerance, shootingPoseTolerance)) { //Shooting stuff
//                    open.schedule()
//                    intake.schedule()
//                    if (pusherSetUp5) {
//                        pusherSetUp5 = false
//                        actionTimer!!.resetTimer()
//                    }
//                    PanelsTelemetry.telemetry.addData("actionTimer", actionTimer!!.elapsedTimeSeconds)
//                    PanelsTelemetry.telemetry.addData("pathTimer", pathTimer!!.elapsedTimeSeconds)
//                    if (actionTimer!!.elapsedTimeSeconds >= delay3rdBall + afterPushDelay) {
//                        out.schedule()
//                        setPathState(AutonPath.EndAuton)
//                    } else if (actionTimer!!.elapsedTimeSeconds >= delay3rdBall) {
//                        `in`.schedule()
//                    }
//                }
//            }

            AutonPath.EndAuton -> if (!follower!!.isBusy) {
                follower!!.followPath(robotGoToShoot4!!)
                if (follower!!.atPose(endPose, 1.25, 1.25)) {
                    follower!!.pausePathFollowing()
                    follower!!.breakFollowing()
                    follower!!.setMaxPower(0.0)
                }
            }
        }
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play".  */
    override fun onUpdate() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower!!.update();
//        forward.schedule()

        autonomousPathUpdate()
        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState)
        telemetry.addData("x", follower!!.pose.x)
        telemetry.addData("y", follower!!.pose.y)
        telemetry.addData("heading", follower!!.pose.heading)
        telemetry.update()

        PanelsTelemetry.telemetry.update()
    }

    /** This method is called continuously after Init while waiting for "play". **/
    override fun onInit() {
        follower = Constants.createFollower(hardwareMap)
        buildPaths()
        follower!!.setStartingPose(startPose)
    }

    /** This method is called continuously after Init while waiting for "play".  */
    override fun onWaitForStart() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system  */
    override fun onStartButtonPressed() {
        ShooterSubsystem.off()
        open.schedule()
        PusherServoSubsystem.out()
        MagazineServoSubsystem.stop()
        opmodeTimer!!.resetTimer()
        actionTimer!!.resetTimer()
        setPathState(AutonPath.RobotShoot1)

        val thetaAim = TurretThetaSubsystem.AutoAim(
            {
                hypot(
                    distanceGoalX - follower!!.pose.x,
                    distanceGoalY - follower!!.pose.y,
                )
            },
            { (-m!!*it+70.67).coerceIn(55.0, 63.0).deg }
        )
        thetaAim.schedule();

        val autoAimPhi = TurretPhiSubsystem.AutoAim(
            { directionGoalX - follower!!.pose.x },
            { directionGoalY - follower!!.pose.y },
            { follower!!.pose.heading.rad }
        );
        autoAimPhi.schedule();

        val shooterAutoAim = ShooterSubsystem.AutoAim(
            { hypot(distanceGoalX - follower!!.pose.x, distanceGoalY - follower!!.pose.y) },
            { (578 + 12.7*it + -0.0921*it*it + 0.000316*it*it*it).coerceIn(0.0, 1500.0) }
        )
        shooterAutoAim.schedule()
    }

    /** We do not use this because everything should automatically disable  */
    override fun onStop() {
        val file = File(filePath)
        file.writeText(
            follower!!.pose.x.toString() + "\n" +
                    follower!!.pose.y.toString() + "\n" +
                    follower!!.pose.heading.toString() + "\n"
        )
    }
}
