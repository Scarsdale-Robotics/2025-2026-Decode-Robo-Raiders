package org.firstinspires.ftc.teamcode.Auton
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathChain
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
//import org.firstinspires.ftc.teamcode.opmodes.testing.TeleOpInProg.Companion.goalX
//import org.firstinspires.ftc.teamcode.opmodes.testing.TeleOpInProg.Companion.goalY
import org.firstinspires.ftc.teamcode.opmodes.testing.TeleOpInProg.Companion.m
import org.firstinspires.ftc.teamcode.opmodes.testing.TeleOpInProg.Companion.overaimSecs
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.LowerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeSubsystem.intake
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeSubsystem.stop
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.MagazineServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.MagazineServoSubsystem.forward
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.MagblockServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.MagblockServoSubsystem.close
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.MagblockServoSubsystem.open
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.PusherServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.PusherServoSubsystem.`in`
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.PusherServoSubsystem.out
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem

import java.io.File
import kotlin.math.hypot

//Auton Naming Convention
//total slots = 4: __ __ __ __
//First slot = Name Type: Auton
//2nd slot = Side type: Blue, Red
//3rd slot = Classifier type (There can be multiple types on the same auto):
//1. Leaving it blank
//2. Wait (only for shooter type autons)
//3. Far (only for shooter and backup type autons)
//4. Close (only for shooter and backup type autons)
//4th slot = Auton type: Motif, Backup, Shooter, Artifact (is just Motif but does not care about motifs)
//Example Auton = AutonBlueCloseBackup, AutonRedWaitFarShooter ...
//Main Autons should be: Auton__WaitFarShooter & Auton__Motif
@Autonomous(name = "Auton Blue Artifact", group = "Auton")
class AutonBlueArtifact : NextFTCOpMode() {
    private var pathTimer: Timer? = null
    var actionTimer: Timer? = null;
    var opmodeTimer: Timer? = null;

    val delay3rdBall: Double = 2.8
    val afterPushDelay: Double = 0.5

    val goalX = 12
    val goalY = 136

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
        RobotIntake3,
        RobotShoot4,
        RobotIntake4,
        RobotShoot5,
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
    private val startPose = Pose(33.0, 136.0, Math.toRadians(180.0)) // Start Pose of our robot.

    private val intake1stLinePos = Pose(11.5, 60.0)
    private val intake1ControlPointPos = Pose(73.0, 52.0)

    private val intake2ndLinePos = Pose(22.0, 84.0)

    private val intake3rdLinePos = Pose(11.5, 36.0)
    private val intake3ControlPointPos = Pose(77.0, 33.0)

    private val intake4thLinePos = Pose(11.0, 11.0, Math.toRadians(200.0))
    private val intake4ControlPointPos = Pose(10.0, 67.0)

    private val shootingPose =
        Pose(57.0, 84.0, Math.toRadians(180.0)) // The shooter position for everything
    private val endPose = Pose(57.0, 84.0, Math.toRadians(0.0)) // End Pose of our robot


    /////////////
    ////Paths////
    /////////////
    // The different paths the robot will take in during Auton
    private var robotShootPreload: PathChain? = null

    private var robotIntake1: PathChain? = null
    private var robotGoToShoot1: PathChain? = null

    private var robotIntake2: PathChain? = null
    private var robotGoToShoot2: PathChain? = null

    private var robotIntake3: PathChain? = null
    private var robotGoToShoot3: PathChain? = null

    private var robotIntake4: PathChain? = null
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
                BezierLine(
                    shootingPose,
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

        //3rd Intake
        robotIntake3 = follower!!.pathBuilder()
            .addPath(
                BezierCurve(
                    shootingPose,
                    intake3ControlPointPos,
                    intake3rdLinePos
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()
        //3rd Go Shoot
        robotGoToShoot3 = follower!!.pathBuilder()
            .addPath(
                BezierLine(
                    intake3rdLinePos,
                    shootingPose
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()

        //4th Intake
        robotIntake4 = follower!!.pathBuilder()
            .addPath(
                BezierCurve(
                    shootingPose,
                    intake4ControlPointPos,
                    intake4thLinePos
                )
            )
            .setLinearHeadingInterpolation(
                shootingPose.heading,
                intake4thLinePos.heading
            )
            .build()
        //4th Go Shoot
        robotGoToShoot4 = follower!!.pathBuilder()
            .addPath(
                BezierLine(
                    intake4thLinePos,
                    endPose
                )
            )
            .setLinearHeadingInterpolation(
                intake4thLinePos.heading,
                endPose.heading
            )
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
                if (follower!!.atPose(shootingPose, 0.15, 0.15)) { //Shooting stuff
                    open.schedule()
                    intake.schedule()
                    if (pusherSetUp1) {
                        pusherSetUp1 = false
                        actionTimer!!.resetTimer()
                    }
                    PanelsTelemetry.telemetry.addData("actionTimer", actionTimer!!.elapsedTimeSeconds)
                    PanelsTelemetry.telemetry.addData("pathTimer", pathTimer!!.elapsedTimeSeconds)
                    if (actionTimer!!.elapsedTimeSeconds >= delay3rdBall + afterPushDelay) {
                        out.schedule()
                        setPathState(AutonPath.RobotIntake1)
                    } else if (actionTimer!!.elapsedTimeSeconds >= delay3rdBall) {
                        `in`.schedule()
                    }
                }
            }

            AutonPath.RobotIntake1 -> if (!follower!!.isBusy) {
                follower!!.setMaxPower(0.65)
                follower!!.followPath(robotIntake1!!)
                setPathState(AutonPath.RobotShoot2)
            }

            AutonPath.RobotShoot2 -> if (!follower!!.isBusy) {
                follower!!.setMaxPower(1.0)
                if (pathTimer!!.elapsedTimeSeconds > 3.33) { //hopefully 2 ball already in mag
                    close.schedule()
                }
                if (pathF2) {
                    follower!!.followPath(robotGoToShoot1!!)
                    pathF2 = false
                }
                if (follower!!.atPose(shootingPose, 0.15, 0.15)) { //Shooting stuff
                    open.schedule()
                    intake.schedule()
                    if (pusherSetUp2) {
                        pusherSetUp2 = false
                        actionTimer!!.resetTimer()
                    }
                    PanelsTelemetry.telemetry.addData("actionTimer", actionTimer!!.elapsedTimeSeconds)
                    PanelsTelemetry.telemetry.addData("pathTimer", pathTimer!!.elapsedTimeSeconds)
                    if (actionTimer!!.elapsedTimeSeconds >= delay3rdBall + afterPushDelay) {
                        out.schedule()
                        setPathState(AutonPath.RobotIntake2)
                    } else if (actionTimer!!.elapsedTimeSeconds >= delay3rdBall) {
                        `in`.schedule()
                    }
                }
            }

            AutonPath.RobotIntake2 -> if (!follower!!.isBusy) {
                follower!!.setMaxPower(0.75)
                follower!!.followPath(robotIntake2!!)
                setPathState(AutonPath.RobotShoot3)
            }

            AutonPath.RobotShoot3 -> if (!follower!!.isBusy) {
                follower!!.setMaxPower(1.0)
                if (pathTimer!!.elapsedTimeSeconds > 1) { //hopefully 1 ball already in mag
                    close.schedule()
                }
                if (pathF3) {
                    follower!!.followPath(robotGoToShoot2!!)
                    pathF3 = false
                }
                if (follower!!.atPose(shootingPose, 0.2, 0.2)) { //Shooting stuff
                    open.schedule()
                    intake.schedule()
                    if (pusherSetUp3) {
                        pusherSetUp3 = false
                        actionTimer!!.resetTimer()
                    }
                    PanelsTelemetry.telemetry.addData("actionTimer", actionTimer!!.elapsedTimeSeconds)
                    PanelsTelemetry.telemetry.addData("pathTimer", pathTimer!!.elapsedTimeSeconds)
                    if (actionTimer!!.elapsedTimeSeconds >= delay3rdBall + afterPushDelay) {
                        out.schedule()
                        setPathState(AutonPath.EndAuton)
                    } else if (actionTimer!!.elapsedTimeSeconds >= delay3rdBall) {
                        `in`.schedule()
                    }
                }
            }

            AutonPath.RobotIntake3 -> if (!follower!!.isBusy) {
                follower!!.setMaxPower(0.5)
                follower!!.followPath(robotIntake3!!)
                setPathState(AutonPath.RobotShoot4)
            }

            AutonPath.RobotShoot4 -> if (!follower!!.isBusy) {
                follower!!.setMaxPower(1.0)
                if (pathF4) {
                    follower!!.followPath(robotGoToShoot3!!)
                    pathF4 = false
                }
                if (pathTimer!!.elapsedTimeSeconds > 0.5 && follower!!.atPose(shootingPose, 0.15, 0.15)) {
                    open.schedule()
                    stop.schedule()
                    `in`.schedule()
                }
                if (pathTimer!!.elapsedTimeSeconds > 3) {
                    intake.schedule()
                    close.schedule()
                    out.schedule()
                    setPathState(AutonPath.RobotIntake4)
                }
            }

            AutonPath.RobotIntake4 -> if (!follower!!.isBusy) {
                follower!!.setMaxPower(0.35)
                follower!!.followPath(robotIntake4!!)
                setPathState(AutonPath.RobotShoot5)
            }

            AutonPath.RobotShoot5 -> if (!follower!!.isBusy) {
                follower!!.setMaxPower(1.0)
                if (pathF5) {
                    follower!!.followPath(robotGoToShoot4!!)
                    pathF5 = false
                }
                if (pathTimer!!.elapsedTimeSeconds > 0.5 && follower!!.atPose(shootingPose, 0.15, 0.15)) {
                    open.schedule()
                    stop.schedule()
                    `in`.schedule()
                }
                if (pathTimer!!.elapsedTimeSeconds > 3) {
                    setPathState(AutonPath.EndAuton)
                    if (pathTimer!!.elapsedTimeSeconds > 1 && follower!!.atPose(shootingPose, 0.15, 0.15)) {
                        close.schedule()
                        out.schedule()
                    }
                }
            }

            AutonPath.EndAuton -> {
//                val file = File("TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RobotAutonEndPos")
//                file.writeText(
//                    follower!!.pose.x.toString() + "\n" +
//                            follower!!.pose.y.toString() + "\n" +
//                            follower!!.pose.heading.toString())
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
        MagblockServoSubsystem.close()
        PusherServoSubsystem.out();
        MagazineServoSubsystem.stop()
        opmodeTimer!!.resetTimer()
        actionTimer!!.resetTimer()
        setPathState(AutonPath.RobotShoot1)

        val thetaAim = TurretThetaSubsystem.AutoAim(
            {
                hypot(
                    goalX - follower!!.pose.x,
                    goalY - follower!!.pose.y,
                )
            },
            { (-m!!*it+70.67).coerceIn(55.0, 63.0).deg }
        )
        thetaAim.schedule();

        val autoAimPhi = TurretPhiSubsystem.AutoAim(
            { goalX - follower!!.pose.x },
            { goalY - follower!!.pose.y },
            { follower!!.pose.heading.rad }
        );
        autoAimPhi.schedule();

        val shooterAutoAim = ShooterSubsystem.AutoAim(
            { hypot(goalX - follower!!.pose.x, goalY - follower!!.pose.y) },
            { (578 + 12.7*it + -0.0921*it*it + 0.000316*it*it*it).coerceIn(0.0, 1500.0) }
        )
        shooterAutoAim.schedule()
    }

    /** We do not use this because everything should automatically disable  */
    override fun onStop() {}  // todo: log to file
}
