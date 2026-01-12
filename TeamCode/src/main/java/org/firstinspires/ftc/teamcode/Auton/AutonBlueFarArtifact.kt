package org.firstinspires.ftc.teamcode.Auton

import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.pedropathing.util.Timer
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.opmodes.testing.TeleOpInProg.Companion.m
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.LowerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.MagazineServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.MagblockServoSubsystem.open
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.PusherServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem
import org.firstinspires.ftc.teamcode.utils.Lefile.filePath
import java.io.File
import kotlin.time.Duration.Companion.seconds
import kotlin.math.hypot
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.extensions.pedro.PedroComponent

//Auton Naming Convention
//total slots = 4: __ __ __ __
//First slot = Name Type: Auton
//2nd slot = Side type: Blue, Red
//3rd slot = Classifier type (There can be multiple types on the same auto):
//1. Leaving it blank
//2. Wait (only for shooter type autons)
//3. Far (only for shooter and backup type autons)
//4. Close (only for shooter and backup type autons)
//4th slot = Auton type: Motif, Backup, Shooter, Artifact, CoOp
//Example Auton = AutonBlueCloseBackup, AutonRedWaitFarShooter ...
//Main Autons should be: Auton__ __Artifact & Auton__ __ CoOp
class AutonBlueFarArtifact: NextFTCOpMode(){ //Pretend robot is 14 to 16 (14 is intake to backplate)
    //////////////////////
    ////Base Variables////
    //////////////////////
    private var pathTimer: Timer? = null
    var actionTimer: Timer? = null;
    var opmodeTimer: Timer? = null;
    init { addComponents(
        PedroComponent(Constants::createFollower),
        SubsystemComponent(
            LowerSubsystem,
            OuttakeSubsystem));
        actionTimer = Timer()
        pathTimer = Timer()
        opmodeTimer = Timer()
        opmodeTimer!!.resetTimer()
    }
    var follower : Follower? = null
    /////////////////
    ////Constants////
    /////////////////
    companion object {
        val delay3rdBall: Double = 2.5
        val afterPushDelay: Double = 0.2

        val distanceGoalX = 12
        val distanceGoalY = 132
        var directionGoalX = 4.0;
        var directionGoalY = 144.0-4.0;

        private val toleranceIntakeMagSeq = 5.0

        private var magBallHitDelay = 1.0  // pessimistic time to hit magblock
        private var magBallEnterDelay = magBallHitDelay + 2.5  // time to pass magblock

        private var intakeMaxPower = 1.0
        private var shootReturnPower = 1.0
        private var delayAfterIntake = 0.42

        private var delayOut = 0.0;

        private var intakeMagblockDelay = 0.2;

        private var intakeEndPosTolerance = 2.0;
        private var shootingPoseTolerance = 3.0;
    }


    /////////////////
    ////Positions////
    /////////////////
    //Constant positions
    private val startPose = Pose(55.0, 8.0, Math.toRadians(180.0)) // Start Pose of our robot.
    private val shootPoseClose = Pose(54.5, 75.9, Math.toRadians(180.0)) // Close Shoot Pose of our robot.
    private val shootPoseFar = Pose(55.4, 18.5, Math.toRadians(180.0)) // Far Shoot Pose of our robot.
    private val gateOpenPose = Pose(11.5, 60.48, Math.toRadians(145.0)) // Gate Open Pose of our robot.

    private val commonIntakePos = Pose(10.92, 10.5, Math.toRadians(180.0))
    private val commonIntakeControlPos = Pose(54.8, 36.7)

    private val parkPose = Pose (35.5, 18.5, Math.toRadians(180.0))

    // Non-constant positions
    private val intake1Pos = Pose(25.0, 36.0) // Intake Pos1
    private val intake1ControlPos = Pose(48.4, 32.0)

    private val intake2Pos = Pose(25.0, 60.0) // Intake Pos2
    private val intake2ControlPos = Pose(58.9, 51.3)

    private val intake3Pos = Pose(25.0, 84.0) // Intake Pos3




    /////////////
    ////Paths////
    /////////////
    // The different paths the robot will take in during Auton
    private var robotShootPreload: PathChain? = null //DO NOT NEED

    private var robotIntake1: PathChain? = null
    private var robotGoToShoot1: PathChain? = null

    private var robotIntake2: PathChain? = null
    private var robotGoToShoot2: PathChain? = null

    private var robotOpenLeverFromFar: PathChain? = null
    private var robotOpenLeverFromClose: PathChain? = null
    private var LeverGoShoot: PathChain? = null

    private var robotIntake3: PathChain? = null
    private var robotGoToShoot3: PathChain? = null

    private var robotIntake4: PathChain? = null
    private var robotGoToShoot4: PathChain? = null

    private var robotPark: PathChain? = null


    ////////////////////
    ////Path Builder////
    ////////////////////
    fun buildPaths() {
        //1st Intake
        robotIntake1 = follower!!.pathBuilder()
            .addPath(
                BezierCurve(
                    startPose,
                    intake1ControlPos,
                    intake1Pos
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()
        //1st Go Shoot
        robotGoToShoot1 = follower!!.pathBuilder()
            .addPath(
                BezierLine(
                    intake1Pos,
                    shootPoseFar
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()
        //2nd Intake
        robotIntake2 = follower!!.pathBuilder()
            .addPath(
                BezierCurve(
                    shootPoseFar,
                    intake2ControlPos,
                    intake2Pos
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()
        //2nd Go Shoot
        robotGoToShoot2 = follower!!.pathBuilder()
            .addPath(
                BezierLine(
                    intake2Pos,
                    shootPoseFar
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()
        //1st Go to Gate
        robotOpenLeverFromFar = follower!!.pathBuilder()
            .addPath(
                BezierLine(
                    shootPoseFar,
                    gateOpenPose
                )
            )
            .setConstantHeadingInterpolation(gateOpenPose.heading)
            .build()
        //Lever Go Shoot
        LeverGoShoot = follower!!.pathBuilder()
            .addPath(
                BezierLine(
                    gateOpenPose,
                    shootPoseClose
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()
        //3rd Intake
        robotIntake3 = follower!!.pathBuilder()
            .addPath(
                BezierLine(
                    shootPoseClose,
                    intake3Pos
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()
        //3rd Go Shoot
        robotGoToShoot3 = follower!!.pathBuilder()
            .addPath(
                BezierLine(
                    intake3Pos,
                    shootPoseClose
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()
        //2nd Go to Gate
        robotOpenLeverFromClose = follower!!.pathBuilder()
            .addPath(
                BezierLine(
                    shootPoseClose,
                    gateOpenPose
                )
            )
            .setConstantHeadingInterpolation(gateOpenPose.heading)
            .build()
        //4th Intake
        robotIntake4 = follower!!.pathBuilder()
            .addPath(
                BezierCurve(
                    shootPoseClose,
                    commonIntakeControlPos,
                    commonIntakePos
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()
        //4th Go Shoot
        robotGoToShoot4 = follower!!.pathBuilder()
            .addPath(
                BezierLine(
                    commonIntakePos,
                    shootPoseFar
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()
        //Go Park
        robotPark = follower!!.pathBuilder()
            .addPath(
                BezierLine(
                    shootPoseFar,
                    parkPose
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()
    }

    /////////////////////////
    ////Autonomous Runner////
    /////////////////////////
    private val autonomousRoutine: Command
        get() = SequentialGroup( //Go to first shoot
//            Lift.toHigh,
            ParallelGroup(
//                Lift.toMiddle,
//                Claw.close
            ),
            Delay(0.5),
            ParallelGroup(
//                Claw.open,
//                Lift.toLow
            )
        )

    override fun onUpdate() {
        // These loop the movements of the robot, these must be called continuously in order to work
        follower!!.update();
//        forward.schedule()

        autonomousRoutine()
        // Feedback to Driver Hub for debugging
//        telemetry.addData("path state", pathState)
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
//        setPathState(AutonPath.RobotShoot1)

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