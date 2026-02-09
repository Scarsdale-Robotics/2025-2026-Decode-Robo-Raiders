package org.firstinspires.ftc.teamcode.Auton.MainAutons
import org.firstinspires.ftc.teamcode.Auton.AutonPositions

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

@Autonomous(name = "Auton Blue Close Artifact", group = "Auton")
@Configurable
class AutonBlueCloseArtifact: NextFTCOpMode() {
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

    companion object {
        val delayStartShoot: Double = 1.0
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
    private val shootPoseCloseFromIntake1 = Pose(57.0, 76.6, Math.toRadians(128.0)) // Close Shoot Pose of our robot.


    /////////////
    ////Paths////
    /////////////
    // The different paths the robot will take in during Auton
    private var robotShootPreload: PathChain? = null
    private var robotGoToShoot1: PathChain? = null //since preload also intakes, there is no intake1

    private var robotIntake2: PathChain? = null
    private var robotGoToShoot2: PathChain? = null

    private var robotOpenLeverFromFar: PathChain? = null
    private var robotOpenLeverFromClose: PathChain? = null
    private var robotBackupFromRamp: PathChain? = null
    private var LeverGoShoot: PathChain? = null

    private var robotIntake3: PathChain? = null
    private var robotGoToShoot3: PathChain? = null

//    private var robotIntake4: PathChain? = null
//    private var robotGoToShoot4: PathChain? = null
    private var robotPark: PathChain? = null

    ////////////////////
    ////Path Builder////
    ////////////////////
    fun buildPaths() {
        //Shoots preload and intakes once
        robotShootPreload = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    AutonPositions.Blue(AutonPositions.startPoseClose),
                    AutonPositions.Blue(AutonPositions.startAutonControlPos),
                    AutonPositions.Blue(AutonPositions.intake2Pos),
                )
            )
            .setConstantHeadingInterpolation(AutonPositions.Blue(AutonPositions.shootPoseClose).heading)
            .build()

        //1st Go Shoot
        robotGoToShoot1 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.intake2Pos),
                    AutonPositions.Blue(AutonPositions.shootPoseClose)
                )
            )
            .setConstantHeadingInterpolation(AutonPositions.Blue(AutonPositions.shootPoseClose).heading)
            .build()

        // Go to Gate
        robotOpenLeverFromFar = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.shootPoseClose),
                    AutonPositions.Blue(AutonPositions.gateOpenPose),
                )
            )
            .setConstantHeadingInterpolation(AutonPositions.Blue(AutonPositions.gateOpenPose).heading)
            .build()

        //backs up from lever to stay legal
        robotBackupFromRamp = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.gateOpenPose),
                    AutonPositions.Blue(AutonPositions.gateAfterOpenPose),
                )
            )
            .setConstantHeadingInterpolation(AutonPositions.Blue(AutonPositions.gateAfterOpenPose).heading)
            .build()

        //Lever Go Shoot
        LeverGoShoot = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.gateAfterOpenPose),
                    AutonPositions.Blue(AutonPositions.shootPoseClose)
                )
            )
            .setConstantHeadingInterpolation(AutonPositions.Blue(AutonPositions.shootPoseClose).heading)
            .build()

        //2nd Intake
        robotIntake2 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.shootPoseClose),
                    AutonPositions.Blue(AutonPositions.intake1Pos)
                )
            )
            .setConstantHeadingInterpolation(AutonPositions.Blue(AutonPositions.intake1Pos).heading)
            .build()

        //2nd Go Shoot
        robotGoToShoot2 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.intake1Pos),
                    AutonPositions.Blue(AutonPositions.shootPoseClose)
                )
            )
            .setConstantHeadingInterpolation(AutonPositions.Blue(AutonPositions.shootPoseClose).heading)
            .build()

        //3rd Intake
        robotIntake3 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    AutonPositions.Blue(AutonPositions.shootPoseClose),
                    AutonPositions.Blue(AutonPositions.intake3ControlPos),
                    AutonPositions.Blue(AutonPositions.intake3Pos)
                )
            )
            .setConstantHeadingInterpolation(AutonPositions.Blue(AutonPositions.intake3Pos).heading)
            .build()

        //3rd Go Shoot
        robotGoToShoot3 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.intake3Pos),
                    AutonPositions.Blue(AutonPositions.shootPoseClose)
                )
            )
            .setConstantHeadingInterpolation(AutonPositions.Blue(AutonPositions.intake3Pos).heading)
            .build()
        //Go Park
        robotPark = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.shootPoseClose),
                    AutonPositions.Blue(AutonPositions.autonParkPose)
                )
            )
            .setConstantHeadingInterpolation(AutonPositions.Blue(AutonPositions.autonParkPose).heading)
            .build()
    }

    ///////////////////////////
    ////Main Auton Commands////
    ///////////////////////////
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

    ///FIX AUTNOMOUS RUNNER
    /////////////////////////
    ////Autonomous Runner////
    /////////////////////////
    private val autonomousRoutine: Command
        get() = SequentialGroup(
            //Main Group
            SequentialGroup( //Shoots PRELOAD
                ParallelGroup(
//                    TurretPhiSubsystem.SetTargetPhi(5.075.rad),
                    FollowPath(robotShootPreload!!),
                    Delay(delayStartShoot),
                ),
                ShootCommand,
                Delay(delayAfterEachShoot),
                IntakeCommand,
//                FollowPath(robotIntake1!!), //robot goes to intake
                Delay(DelayAfterIntake),
            ),

            ParallelGroup( //Robot goes back to FAR Shoot Position
//                TurretPhiSubsystem.SetTargetPhi((- (-5.075 + 2.0 * PI - PI / 3.0 - PI / 48.0)).rad),
                SequentialGroup(
                    Delay(DelayInIntake),
                    TravelCommand,
                ),
                FollowPath(robotGoToShoot1!!)
            ),

            //////////////////Lever Portion//////////////////
            //////////////////Lever Portion//////////////////
            //////////////////Lever Portion//////////////////
            SequentialGroup( //Shoots FIRST Intake, goes to intake from the lever
                Delay(DelayBeforeShoot),
                ShootCommand,
                Delay(delayAfterEachShoot),
                IntakeCommand,
                FollowPath(robotOpenLeverFromFar!!), //robot goes to intake
                Delay(DelayAtLever),
                FollowPath(robotBackupFromRamp!!),
                Delay(DelayFromRampIntake),
            ),

            ParallelGroup( //Robot goes back to FAR Shoot Position
                SequentialGroup(
                    Delay(DelayInIntake),
                    TravelCommand,
                ),
                FollowPath(LeverGoShoot!!)
            ),

            SequentialGroup( //Shoots SECOND Intake, goes to intake from the lever
                Delay(DelayBeforeShoot),
                ShootCommand,
                Delay(delayAfterEachShoot),
                IntakeCommand,
                FollowPath(robotOpenLeverFromFar!!), //robot goes to intake
                Delay(DelayAtLever),
                FollowPath(robotBackupFromRamp!!),
                Delay(DelayFromRampIntake),
            ),

            ParallelGroup( //Robot goes back to FAR Shoot Position
                SequentialGroup(
                    Delay(DelayInIntake),
                    TravelCommand,
                ),
                FollowPath(LeverGoShoot!!)
            ),

            SequentialGroup( //Shoots THIRD Intake, goes to intake from the lever
                Delay(DelayBeforeShoot),
                ShootCommand,
                Delay(delayAfterEachShoot),
                IntakeCommand,
                FollowPath(robotOpenLeverFromFar!!), //robot goes to intake
                Delay(DelayAtLever),
                FollowPath(robotBackupFromRamp!!),
                Delay(DelayFromRampIntake),
            ),

            ParallelGroup( //Robot goes back to FAR Shoot Position
                SequentialGroup(
                    Delay(DelayInIntake),
                    TravelCommand,
                ),
                FollowPath(LeverGoShoot!!)
            ),

            //////////////////Lever Portion//////////////////
            //////////////////Lever Portion//////////////////
            //////////////////Lever Portion//////////////////

            SequentialGroup( //Shoots FOURTH Intake, goes to intake
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

            SequentialGroup( //Shoots FIFTH Intake, goes to intake
                Delay(DelayBeforeShoot),
                ShootCommand,
                Delay(delayAfterEachShoot),
                IntakeCommand,
                FollowPath(robotIntake3!!), //robot goes to intake
                Delay(DelayAfterIntake),
            ),

            ParallelGroup( //Robot goes back to FAR Shoot Position
                SequentialGroup(
                    Delay(DelayInIntake),
                    TravelCommand,
                ),
                FollowPath(robotGoToShoot3!!)
            ),

            SequentialGroup( //Shoots SIXTH Intake, goes to PARK
                Delay(DelayBeforeShoot),
                ShootCommand,
                Delay(delayAfterEachShoot),
                TravelCommand,
                FollowPath(robotPark!!), //robot goes to intake
            ),
        )

    override fun onUpdate() {
        val dx = AutonBlueFarArtifact.Companion.goalX - PedroComponent.follower.pose.x
        val dy = AutonBlueFarArtifact.Companion.goalY - PedroComponent.follower.pose.y
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

        PedroComponent.follower.setStartingPose(AutonPositions.Blue(AutonPositions.startPoseClose))
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