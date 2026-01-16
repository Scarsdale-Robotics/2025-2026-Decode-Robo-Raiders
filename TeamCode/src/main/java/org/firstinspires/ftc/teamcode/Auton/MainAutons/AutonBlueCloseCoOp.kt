package org.firstinspires.ftc.teamcode.Auton.MainAutons

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.LowerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.LowerMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagblockServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem
import org.firstinspires.ftc.teamcode.utils.Lefile
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
//4th slot = Auton type: Motif, Backup, Shooter, Artifact, CoOp
//Example Auton = AutonBlueCloseBackup, AutonRedWaitFarShooter ...
//Main Autons should be: Auton__ __Artifact & Auton__ __ CoOp
@Autonomous(name = "Auton Blue Close CoOp", group = "Auton")
@Configurable
class AutonBlueCloseCoOp: NextFTCOpMode(){ //Pretend robot is 14 to 16 (14 is intake to backplate)
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
    var follower : Follower? = null
    /////////////////
    ////Constants////
    /////////////////
    companion object {
        val delayStartShoot: Double = 0.01
        val delayAfterEachShoot: Double = 2.0 //currently at a really high #
        val DelayFromRampIntake: Double = 2.0
        val DelayAtLever: Double = 0.1

        val distanceGoalX = 12
        val distanceGoalY = 132
        var directionGoalX = 4.0;
        var directionGoalY = 144.0-4.0;

    }


    /////////////////
    ////Positions////
    /////////////////
    //Constant positions
    private val startPose = Pose(31.6, 135.8, Math.toRadians(180.0)) // Start Pose of our robot.
    private val shootPoseClose =
        Pose(54.5, 75.9, Math.toRadians(180.0)) // Close Shoot Pose of our robot.
//    private val shootPoseFar =
//        Pose(55.4, 18.5, Math.toRadians(180.0)) // Far Shoot Pose of our robot.
    private val gateOpenPose =
        Pose(11.5, 60.48, Math.toRadians(145.0)) // Gate Open Pose of our robot.
    private val gateAfterOpenPose = //F FTC MADE OUR MAIN STRATEGY ILLEGAL
        Pose(11.5, 56.48, Math.toRadians(145.0)) // Gate Open Pose of our robot.

    private val commonIntakePos = Pose(10.92, 10.5, Math.toRadians(180.0))
    private val commonIntakeControlPos = Pose(54.8, 36.7)

    private val parkPose = Pose(45.0, 75.9, Math.toRadians(180.0))

    // Non-constant positions
    private val intake1Pos = Pose(25.0, 60.0) // Intake Pos1

    private val intake2Pos = Pose(25.0, 36.0) // Intake Pos2

    /////////////
    ////Paths////
    /////////////
    // The different paths the robot will take in during Auton
    private var robotShootPreload: PathChain? = null //DO NOT NEED

    private var robotIntake1: PathChain? = null
    private var robotGoToShoot1: PathChain? = null

    private var robotIntake2: PathChain? = null
    private var robotGoToShoot2: PathChain? = null

    private var robotOpenLeverFromClose: PathChain? = null
    private var robotBackupFromRamp: PathChain? = null
    private var LeverGoShoot: PathChain? = null

    private var robotIntakeCommon: PathChain? = null
    private var robotGoToShootCommon: PathChain? = null
    private var robotPark: PathChain? = null

    ////////////////////
    ////Path Builder////
    ////////////////////
    fun buildPaths() {
        //1st Intake
        robotIntake1 = follower!!.pathBuilder()
            .addPath(
                BezierLine(
                    startPose,
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
                    shootPoseClose
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()
        //1st Go to Gate
        robotOpenLeverFromClose = follower!!.pathBuilder()
            .addPath(
                BezierLine(
                    shootPoseClose,
                    gateOpenPose
                )
            )
            .setConstantHeadingInterpolation(gateOpenPose.heading)
            .build()
        //backs up from lever to stay legal
        robotBackupFromRamp = follower!!.pathBuilder()
            .addPath(
                BezierLine(
                    gateOpenPose,
                    gateAfterOpenPose
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
        //Intake 2
        robotIntake2 = follower!!.pathBuilder()
            .addPath(
                BezierLine(
                    shootPoseClose,
                    intake2Pos
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()
        //Robot Go Shoot 2
        robotGoToShoot2 = follower!!.pathBuilder()
            .addPath(
                BezierLine(
                    intake2Pos,
                    shootPoseClose
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()
        //Go Park
        robotPark = follower!!.pathBuilder()
            .addPath(
                BezierLine(
                    shootPoseClose,
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
        get() = SequentialGroup( //Main Group
            SequentialGroup( //Shoots PRELOAD
                Delay(delayStartShoot),
                MagblockServoSubsystem.unblock, //blocker unblock
                Delay(delayAfterEachShoot),
                ParallelGroup(
                    LowerMotorSubsystem.intake, //kebab spinny
                    IntakeServoSubsystem.up //kebab up
                ),
                FollowPath(robotIntake1!!) //robot goes to intake
            ),

            SequentialGroup( //Robot goes back to CLOSE Shoot Position
                ParallelGroup(
                    LowerMotorSubsystem.off, //kebab stop spinny
                    IntakeServoSubsystem.down //kebab down
                ),
                FollowPath(robotGoToShoot1!!)
            ),

            SequentialGroup( //Shoots FIRST Intake
                MagblockServoSubsystem.unblock, //blocker unblock
                Delay(delayAfterEachShoot),
                ParallelGroup(
                    LowerMotorSubsystem.intake, //kebab spinny
                    IntakeServoSubsystem.up //kebab up
                ),
                FollowPath(robotOpenLeverFromClose!!) //robot goes to intake
            ),

            SequentialGroup( //Intakes from RAMP and then moves to CLOSE Shoot Position
                Delay(AutonBlueFarArtifact.Companion.DelayAtLever),
                FollowPath(robotBackupFromRamp!!),
                Delay(DelayFromRampIntake),
                ParallelGroup(
                    LowerMotorSubsystem.off, //kebab stop spinny
                    IntakeServoSubsystem.down //kebab down
                ),
                FollowPath(LeverGoShoot!!), //robot goes to level
            ),

            SequentialGroup( //Shoots SECOND Intake
                MagblockServoSubsystem.unblock, //blocker unblock
                Delay(AutonBlueFarArtifact.Companion.delayAfterEachShoot),
                ParallelGroup(
                    LowerMotorSubsystem.intake, //kebab spinny
                    IntakeServoSubsystem.up //kebab up
                ),
                FollowPath(robotIntake2!!), //robot goes to intake
            ),

            SequentialGroup( //Robot goes back to CLOSE Shoot Position
                ParallelGroup(
                    LowerMotorSubsystem.off, //kebab stop spinny
                    IntakeServoSubsystem.down //kebab down
                ),
                FollowPath(robotGoToShoot2!!)
            ),
            //////////////////////REPEATABLE SECTION//////////////////////
            SequentialGroup( //Intakes from RAMP and then moves to CLOSE Shoot Position
                Delay(AutonBlueFarArtifact.Companion.DelayAtLever),
                FollowPath(robotBackupFromRamp!!),
                Delay(DelayFromRampIntake),
                ParallelGroup(
                    LowerMotorSubsystem.off, //kebab stop spinny
                    IntakeServoSubsystem.down //kebab down
                ),
                FollowPath(LeverGoShoot!!), //robot goes to level
            ),

            SequentialGroup( //Shoots Common Intake #1
                MagblockServoSubsystem.unblock, //blocker unblock
                Delay(delayAfterEachShoot),
                ParallelGroup(
                    LowerMotorSubsystem.intake, //kebab spinny
                    IntakeServoSubsystem.up //kebab up
                ),
                FollowPath(LeverGoShoot!!) //robot goes to intake
            ),
            //////////////////////REPEATABLE SECTION//////////////////////
            SequentialGroup( //Intakes from RAMP and then moves to CLOSE Shoot Position
                Delay(AutonBlueFarArtifact.Companion.DelayAtLever),
                FollowPath(robotBackupFromRamp!!),
                Delay(DelayFromRampIntake),
                ParallelGroup(
                    LowerMotorSubsystem.off, //kebab stop spinny
                    IntakeServoSubsystem.down //kebab down
                ),
                FollowPath(LeverGoShoot!!), //robot goes to level
            ),

            SequentialGroup( //Shoots the COMMON Intake #1
                MagblockServoSubsystem.unblock, //blocker unblock
                Delay(delayAfterEachShoot),
                ParallelGroup(
                    LowerMotorSubsystem.intake, //kebab spinny
                    IntakeServoSubsystem.up //kebab up
                ),
                FollowPath(robotOpenLeverFromClose!!) //robot goes to intake
            ),
            //////////////////////REPEATABLE SECTION//////////////////////
            SequentialGroup( // PARKS YAY
                FollowPath(robotPark!!), //robot goes to PARK
            )
        )

    override fun onUpdate() {
        // These loop the movements of the robot, these must be called continuously in order to work
//        follower!!.update();
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
//        follower = Constants.createFollower(hardwareMap)
        buildPaths()
//        follower!!.setStartingPose(startPose)
    }

    /** This method is called continuously after Init while waiting for "play".  */
    override fun onWaitForStart() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system  */
    override fun onStartButtonPressed() {
        ShooterSubsystem.off()
        IntakeServoSubsystem.down.schedule() //puts kebab into default position, down.
        LowerMotorSubsystem.off.schedule() //sets kebab into its default spin rate, off.
        MagblockServoSubsystem.block.schedule() // puts up magBlocker.

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
            { (/*-m!!**/it+70.67).coerceIn(55.0, 63.0).deg }
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
        val file = File(Lefile.filePath)
        file.writeText(
            follower!!.pose.x.toString() + "\n" +
                    follower!!.pose.y.toString() + "\n" +
                    follower!!.pose.heading.toString() + "\n"
        )
    }
}