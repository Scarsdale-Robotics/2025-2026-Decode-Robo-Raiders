package org.firstinspires.ftc.teamcode.Auton.MainAutons

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.ParallelRaceGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.LowerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem

import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagServoSubsystem
//import org.firstinspires.ftc.teamcode.subsystems.lower.LowerMotorSubsystem

import org.firstinspires.ftc.teamcode.subsystems.lower.MagblockServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem
import org.firstinspires.ftc.teamcode.utils.Lefile
import java.io.File
import kotlin.math.PI
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min

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
@Autonomous(name = "Auton Blue Far Artifact", group = "Auton")
@Configurable
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
            OuttakeSubsystem
        )
    );
        actionTimer = Timer()
        pathTimer = Timer()
        opmodeTimer = Timer()
        opmodeTimer!!.resetTimer()
    }
//    var follower : Follower? = null
    /////////////////
    ////Constants////
    /////////////////
    companion object {
        val delayStartShoot: Double = 3.5
        val DelayBeforeShoot: Double = 0.55
        val delayAfterEachShoot: Double = 2.0 //currently at a really high #
        val DelayFromRampIntake: Double = 1.8
        val DelayInIntake: Double = 1.1
        val DelayAfterIntake: Double = 0.3
        val DelayAtLever: Double = 0.05

        val goalX = 3.0
        val goalY = 144.0 - 3.0
//        var directionGoalX = 4.0;
//        var directionGoalY = 144.0-4.0;

    }


    /////////////////
    ////Positions////
    /////////////////
    //Constant positions
    private val startPose = Pose(56.43, 8.503, Math.toRadians(180.0)) // Start Pose of our robot.
    private val shootPoseClose =
        Pose(57.0, 76.6, Math.toRadians(128.0)) // Close Shoot Pose of our robot.
    private val shootPoseFar =
        Pose(57.0, 13.5, Math.toRadians(120.0)) // Far Shoot Pose of our robot.
    private val gateOpenPose =
        Pose(14.0, 60.0, Math.toRadians(145.0)) // Gate Open Pose of our robot.
    private val gateAfterOpenPose = //F FTC MADE OUR MAIN STRATEGY ILLEGAL
        Pose(14.0, 53.0, Math.toRadians(145.0)) // Gate Open Pose of our robot.

    private val commonIntakePos = Pose(12.5, 10.9, Math.toRadians(180.0))
    private val commonIntakeControlPos = Pose(54.8, 36.7)

    private val parkPose = Pose(40.5, 40.5, Math.toRadians(180.0))

    // Non-constant positions
    private val intake1Pos = Pose(19.0, 40.0) // Intake Pos1
    private val intake1ControlPos = Pose(48.4, 32.0)

    private val intake2Pos = Pose(19.0, 58.0) // Intake Pos2
    private val intake2ControlPos = Pose(58.9, 63.3)

    private val intake3Pos = Pose(21.5, 84.0) // Intake Pos3




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
    private var robotBackupFromRamp: PathChain? = null
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
        robotIntake1 = PedroComponent.follower.pathBuilder()
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
        robotGoToShoot1 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    intake1Pos,
                    shootPoseFar
                )
            )
            .setConstantHeadingInterpolation(shootPoseFar.heading)
            .build()
        //2nd Intake
        robotIntake2 = PedroComponent.follower.pathBuilder()
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
        robotGoToShoot2 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    intake2Pos,
                    shootPoseFar
                )
            )
            .setConstantHeadingInterpolation(shootPoseFar.heading)
            .build()
        //1st Go to Gate
        robotOpenLeverFromFar = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    shootPoseFar,
                    gateOpenPose
                )
            )
            .setConstantHeadingInterpolation(gateOpenPose.heading)
            .build()
        //backs up from lever to stay legal
        robotBackupFromRamp = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    gateOpenPose,
                    gateAfterOpenPose
                )
            )
            .setConstantHeadingInterpolation(gateOpenPose.heading)
            .build()
        //Lever Go Shoot
        LeverGoShoot = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    gateAfterOpenPose,
                    shootPoseClose
                )
            )
            .setConstantHeadingInterpolation(shootPoseClose.heading)
            .build()
        //3rd Intake
        robotIntake3 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    shootPoseClose,
                    intake3Pos
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()
        //3rd Go Shoot
        robotGoToShoot3 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    intake3Pos,
                    shootPoseClose
                )
            )
            .setConstantHeadingInterpolation(shootPoseClose.heading)
            .build()
        //2nd Go to Gate
        robotOpenLeverFromClose = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    shootPoseClose,
                    gateOpenPose
                )
            )
            .setConstantHeadingInterpolation(gateOpenPose.heading)
            .build()
        //4th Intake
        robotIntake4 = PedroComponent.follower.pathBuilder()
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
        robotGoToShoot4 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    commonIntakePos,
                    shootPoseFar
                )
            )
            .setConstantHeadingInterpolation(shootPoseFar.heading)
            .build()
        //Go Park
        robotPark = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    shootPoseFar,
                    parkPose
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()
    }

    val IntakeCommand: Command
        get() = ParallelGroup(
            IntakeMotorSubsystem.intake,
            MagMotorSubsystem.intake,
            MagServoSubsystem.run,
            MagblockServoSubsystem.block
        )
    val TravelCommand: Command
        get() = ParallelGroup(
            IntakeMotorSubsystem.off,
            MagMotorSubsystem.off,
            MagServoSubsystem.stop,
            MagblockServoSubsystem.block
        )
    val ShootCommand: Command
        get() = ParallelGroup(
            MagblockServoSubsystem.unblock,
            MagMotorSubsystem.On(0.85),
            IntakeMotorSubsystem.intake,
            MagServoSubsystem.run
        )

    /////////////////////////
    ////Autonomous Runner////
    /////////////////////////
    private val autonomousRoutine: Command
        get() = SequentialGroup(
            //Main Group

            SequentialGroup( //Shoots PRELOAD
                ParallelGroup(
                    TurretPhiSubsystem.SetTargetPhi(5.075.rad),
                    Delay(delayStartShoot),
                ),
                ShootCommand,
                Delay(delayAfterEachShoot),
                IntakeCommand,
                FollowPath(robotIntake1!!), //robot goes to intake
                Delay(DelayAfterIntake),
            ),

            ParallelGroup( //Robot goes back to FAR Shoot Position
                TurretPhiSubsystem.SetTargetPhi((- (-5.075 + 2.0 * PI - PI / 3.0 - PI / 48.0)).rad),
                SequentialGroup(
                    Delay(DelayInIntake),
                    TravelCommand,
                ),
                FollowPath(robotGoToShoot1!!)
            ),

            SequentialGroup( //Shoots FIRST Intake
                Delay(DelayBeforeShoot),
                ShootCommand,
                Delay(delayAfterEachShoot),
                IntakeCommand,
                FollowPath(robotIntake2!!), //robot goes to intake
                Delay(DelayAfterIntake),
            ),

            ParallelGroup( //Robot goes back to FAR Shoot Position
                SequentialGroup(
                    Delay(DelayInIntake),
                    TravelCommand,
                ),
                FollowPath(robotGoToShoot2!!)
            ),
            SequentialGroup( //Shoots SECOND Intake
                Delay(DelayBeforeShoot),
                ShootCommand,
                Delay(delayAfterEachShoot),
                IntakeCommand,
                FollowPath(robotIntake3!!), //robot goes to the RAMP LEVER
            ),

//            SequentialGroup( //Intakes from RAMP and then moves to CLOSE Shoot Position
//                Delay(DelayAtLever),
//                IntakeCommand,
//                FollowPath(robotBackupFromRamp!!),
//                Delay(DelayFromRampIntake),
//                TravelCommand,
//                FollowPath(LeverGoShoot!!), //robot goes to level
//            ),
//
//            SequentialGroup( //Shoots THIRD Intake

//                Delay(DelayBeforeShoot),
//                ShootCommand,
//                Delay(delayAfterEachShoot),
//                IntakeCommand,
//                FollowPath(robotIntake3!!), //robot goes to intake
//                Delay(DelayAfterIntake),
//            ),

            ParallelGroup( //Robot goes back to CLOSE Shoot Position
                TurretPhiSubsystem.SetTargetPhi((- (-0.0 - PI / 32.0)).rad),
                SequentialGroup(
                    Delay(DelayInIntake),
                    TravelCommand,
                ),
                FollowPath(robotGoToShoot3!!)
            ),

            SequentialGroup( //Shoots FOURTH Intake
                Delay(DelayBeforeShoot),
                ShootCommand,
                Delay(delayAfterEachShoot),
//                IntakeCommand,
                TravelCommand,
                FollowPath(robotPark!!), //robot goes to intake
//                Delay(DelayAfterIntake),
            ),

//            SequentialGroup( //Intakes from RAMP and then moves to CLOSE Shoot Position
//                Delay(DelayAtLever),
//                IntakeCommand,
//                FollowPath(robotBackupFromRamp!!),
//                Delay(DelayFromRampIntake),
//                TravelCommand,
//                FollowPath(LeverGoShoot!!), //robot goes to level
//            ),
//
//            SequentialGroup( //Shoots FIFTH Intake
//                Delay(DelayBeforeShoot),
//                ShootCommand,
//                Delay(delayAfterEachShoot),
//                IntakeCommand,
//                FollowPath(robotIntake4!!), //robot goes to intake
//            ),

//            ParallelGroup( //Robot goes back to CLOSE Shoot Position
////                TurretPhiSubsystem.SetTargetPhi((-2 * PI - (-5.075 + 2.0 * PI - PI / 3.0 - PI / 32.0)).rad),
//                SequentialGroup(
//                    Delay(DelayInIntake),
//                    TravelCommand,
//                ),
//                FollowPath(robotPark!!)
//            ),

//            SequentialGroup( //Shoots SIXTH Intake
//                Delay(DelayBeforeShoot),
//                ShootCommand,
//                Delay(delayAfterEachShoot),
//                TravelCommand,
//                FollowPath(robotPark!!), //robot goes to PARK
//            ),
        )

    override fun onUpdate() {
        val dx = goalX - PedroComponent.follower.pose.x
        val dy = goalY - PedroComponent.follower.pose.y
        val dxy = hypot(dx, dy)
        val dxp = dx + PedroComponent.follower.velocity.xComponent * distanceToTime(dxy)
        val dyp = dy + PedroComponent.follower.velocity.yComponent * distanceToTime(dxy)
        val dxyp = hypot(dxp, dyp)
        val hp = PedroComponent.follower.pose.heading + PedroComponent.follower.velocity.theta * distanceToTime(dxyp)
        ShooterSubsystem.AutoAim(
            dxy,
            { distanceToVelocity(it) }
        )()
//        TurretPhiSubsystem.AutoAim(
//            dx, dy, hp.rad
//        )()
        TurretThetaSubsystem.AutoAim(
            dxyp,
            { distanceToTheta(it) }
        )()

        // These loop the movements of the robot, these must be called continuously in order to work
//        follower!!.update();
//        forward.schedule()
        // Feedback to Driver Hub for debugging
//        telemetry.addData("path state", pathState)
        telemetry.addData("x", PedroComponent.follower.pose.x)
        telemetry.addData("y", PedroComponent.follower.pose.y)
        telemetry.addData("heading", PedroComponent.follower.pose.heading)
        telemetry.update()

        PanelsTelemetry.telemetry.update()
    }

    /** This method is called continuously after Init while waiting for "play". **/
    override fun onInit() {
//        follower = Constants.createFollower(hardwareMap)
        buildPaths()
        ShooterSubsystem.off()
        IntakeMotorSubsystem.off()
        MagMotorSubsystem.off()
        MagServoSubsystem.stop()
        MagblockServoSubsystem.block()

        PedroComponent.follower.setStartingPose(startPose)
    }

    /** This method is called continuously after Init while waiting for "play".  */
    override fun onWaitForStart() {}

    fun distanceToTime(it: Double): Double {
        return it * 0.0;
    }
    fun distanceToVelocity(it: Double): Double {
        return 0.0127 * it * it + 1.81 * it + 937.0;
    }
    fun distanceToTheta(it: Double): Angle {
        return max(min(-0.224*it+74, 63.0), 55.0).deg;
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system  */
    override fun onStartButtonPressed() {
        autonomousRoutine()

        opmodeTimer!!.resetTimer()
        actionTimer!!.resetTimer()
//        setPathState(AutonPath.RobotShoot1)
    }

    /** We do not use this because everything should automatically disable  */
    override fun onStop() {
        val file = File(Lefile.filePath)
        file.writeText(
            PedroComponent.follower.pose.x.toString() + "\n" +
                    PedroComponent.follower.pose.y.toString() + "\n" +
                    PedroComponent.follower.pose.heading.toString() + "\n"
        )
    }
}