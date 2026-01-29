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

import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeMotorSubsystem //spins the Intake
import org.firstinspires.ftc.teamcode.subsystems.lower.MagServoSubsystem //continuous back spinner
import org.firstinspires.ftc.teamcode.subsystems.lower.MagMotorSubsystem //transfer motor
import org.firstinspires.ftc.teamcode.subsystems.lower.MagblockServoSubsystem //MAG BLOCK

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
@Autonomous(name = "Auton Blue Far CoOp", group = "Auton")
@Configurable
class AutonBlueFarCoOp: NextFTCOpMode(){ //Pretend robot is 14 to 16 (14 is intake to backplate)
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
        val delayStartShoot: Double = 0.01
        val delayAfterEachShoot: Double = 2.0 //currently at a really high #

        val distanceGoalX = 12
        val distanceGoalY = 132
        var directionGoalX = 4.0;
        var directionGoalY = 144.0-4.0;

    }


    /////////////////
    ////Positions////
    /////////////////
    //Constant positions
    private val startPose = Pose(55.0, 8.0, Math.toRadians(180.0)) // Start Pose of our robot.
    private val shootPoseClose =
        Pose(54.5, 75.9, Math.toRadians(180.0)) // Close Shoot Pose of our robot.
    private val shootPoseFar =
        Pose(55.4, 18.5, Math.toRadians(180.0)) // Far Shoot Pose of our robot.
    private val gateOpenPose =
        Pose(11.5, 60.48, Math.toRadians(145.0)) // Gate Open Pose of our robot.

    private val commonIntakePos = Pose(10.92, 10.5, Math.toRadians(180.0))
    private val commonIntakeControlPos = Pose(54.8, 36.7)

    private val parkPose = Pose(35.5, 18.5, Math.toRadians(180.0))

    // Non-constant positions
    private val intake1Pos = Pose(25.0, 36.0) // Intake Pos1
    private val intake1ControlPos = Pose(48.4, 32.0)

    private val intake2Pos = Pose(25.0, 60.0) // Intake Pos2
    private val intake2ControlPos = Pose(58.9, 51.3)

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
    private var LeverGoShoot: PathChain? = null

    private var robotIntakeCommon: PathChain? = null
    private var robotGoToShootCommon: PathChain? = null
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
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
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
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
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
        //Lever Go Shoot
        LeverGoShoot = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    gateOpenPose,
                    shootPoseClose
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()
        //Common Intake
        robotIntakeCommon = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    shootPoseFar,
                    commonIntakeControlPos,
                    commonIntakePos
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
            .build()
        //Common Go Shoot
        robotGoToShootCommon = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    commonIntakePos,
                    shootPoseFar
                )
            )
            .setConstantHeadingInterpolation(Math.toRadians(180.0))
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

    /////////////////////////
    ////Autonomous Runner////
    /////////////////////////
    private val autonomousRoutine: Command
        get() = SequentialGroup(
            //Main Group

            SequentialGroup( //Shoots PRELOAD
                Delay(delayStartShoot),
                MagblockServoSubsystem.unblock, //blocker unblock
                IntakeMotorSubsystem.intake, //kebab spinny
                MagMotorSubsystem.intake, // starts mag
                Delay(delayAfterEachShoot),
                ParallelGroup(
//                    LowerMotorSubsystem.intake, //kebab spinny
                ),
                FollowPath(robotIntake1!!) //robot goes to intake
            ),

            SequentialGroup( //Robot goes back to FAR Shoot Position
                ParallelGroup(
                    IntakeMotorSubsystem.off, //kebab stop spinny
                    MagMotorSubsystem.off, // turns off mag
                ),
                FollowPath(robotGoToShoot1!!)
            ),

            SequentialGroup( //Shoots FIRST Intake
                MagblockServoSubsystem.unblock, //blocker unblock
                IntakeMotorSubsystem.intake, //kebab spinny
                MagMotorSubsystem.intake, // starts mag
                Delay(delayAfterEachShoot),
                ParallelGroup(
//                    LowerMotorSubsystem.intake, //kebab spinny
                ),
                FollowPath(robotIntake2!!) //robot goes to intake
            ),

            SequentialGroup( //Robot goes back to FAR Shoot Position
                ParallelGroup(
                    IntakeMotorSubsystem.off, //kebab stop spinny
                    MagMotorSubsystem.off, // turns off mag
                ),
                FollowPath(robotGoToShoot2!!)
            ),

            SequentialGroup( //Shoots the SECOND Intake
                MagblockServoSubsystem.unblock, //blocker unblock
                IntakeMotorSubsystem.intake, //kebab spinny
                MagMotorSubsystem.intake, // starts mag
                Delay(delayAfterEachShoot),
                ParallelGroup(
//                    LowerMotorSubsystem.intake, //kebab spinny
                ),
                FollowPath(robotIntakeCommon!!) //robot goes to intake
            ),

            SequentialGroup( //Robot goes back to FAR Shoot Position
                ParallelGroup(
                    IntakeMotorSubsystem.off, //kebab stop spinny
                    MagMotorSubsystem.off, // turns off mag
                ),
                FollowPath(robotGoToShootCommon!!)
            ),
            //////////////////////REPEATABLE SECTION//////////////////////
            SequentialGroup( //Shoots the COMMON Intake #1
                MagblockServoSubsystem.unblock, //blocker unblock
                IntakeMotorSubsystem.intake, //kebab spinny
                MagMotorSubsystem.intake, // starts mag
                Delay(delayAfterEachShoot),
                ParallelGroup(
//                    LowerMotorSubsystem.intake, //kebab spinny
                ),
                FollowPath(robotIntakeCommon!!) //robot goes to intake
            ),

            SequentialGroup( //Robot goes back to FAR Shoot Position
                ParallelGroup(
                    IntakeMotorSubsystem.off, //kebab stop spinny
                    MagMotorSubsystem.off, // turns off mag
                ),
                FollowPath(robotGoToShootCommon!!)
            ),
            //////////////////////REPEATABLE SECTION//////////////////////
            SequentialGroup( //Shoots the COMMON Intake #2
                MagblockServoSubsystem.unblock, //blocker unblock
                IntakeMotorSubsystem.intake, //kebab spinny
                MagMotorSubsystem.intake, // starts mag
                Delay(delayAfterEachShoot),
                ParallelGroup(
//                    LowerMotorSubsystem.intake, //kebab spinny
                ),
                FollowPath(robotIntakeCommon!!) //robot goes to intake
            ),

            SequentialGroup( //Robot goes back to FAR Shoot Position
                ParallelGroup(
                    IntakeMotorSubsystem.off, //kebab stop spinny
                    MagMotorSubsystem.off, // turns off mag
                ),
                FollowPath(robotGoToShootCommon!!)
            ),
            //////////////////////REPEATABLE SECTION//////////////////////
            SequentialGroup( //Shoots SIXTH Intake
                MagblockServoSubsystem.unblock, //blocker unblock
                IntakeMotorSubsystem.intake, //kebab spinny
                MagMotorSubsystem.intake, // starts mag
                Delay(delayAfterEachShoot),
                ParallelGroup(
//                    LowerMotorSubsystem.intake, //kebab spinny
                ),
                IntakeMotorSubsystem.off, //kebab stop spinny
                FollowPath(robotPark!!), //robot goes to PARK
            )

        )

    override fun onUpdate() {
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
//        follower!!.setStartingPose(startPose)
    }

    /** This method is called continuously after Init while waiting for "play".  */
    override fun onWaitForStart() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system  */
    override fun onStartButtonPressed() {
        ShooterSubsystem.off()
        IntakeMotorSubsystem.off() //sets kebab into its default spin rate, off.
        MagServoSubsystem.run() //sets magSpinner at back into its default spin rate, off.
        MagMotorSubsystem.off() //sets mag into its default spin rate, off.
        MagblockServoSubsystem.block() // puts up magBlocker.

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