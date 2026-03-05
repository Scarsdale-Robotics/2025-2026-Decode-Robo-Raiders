package org.firstinspires.ftc.teamcode.Auton.MainAutons
import org.firstinspires.ftc.teamcode.Auton.AutonPositions
import com.pedropathing.paths.HeadingInterpolator

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
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.NextFTCOpMode

import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.LowerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem

import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagMotorSubsystem
//import org.firstinspires.ftc.teamcode.subsystems.lower.MagServoSubsystem
//import org.firstinspires.ftc.teamcode.subsystems.lower.LowerMotorSubsystem

import org.firstinspires.ftc.teamcode.subsystems.lower.MagblockServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.BORD_Y
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.distAndVeloToThetaClose
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.distAndVeloToThetaFar
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.distanceToTimeClose
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.distanceToTimeFar
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.distanceToVelocityClose
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.distanceToVelocityFar
import org.firstinspires.ftc.teamcode.utils.Lefile
import java.io.File
import kotlin.math.hypot

@Autonomous(name = "[MAIN-21] Auton Blue Close Artifact", group = "Auton")
@Configurable
class AutonBlueCloseArtifact24: NextFTCOpMode() {
    //////////////////////
    ////Base Variables////
    //////////////////////
    private var pathTimer: Timer? = null
    var actionTimer: Timer? = null;
    var opmodeTimer: Timer? = null;
    var pathStarted: Boolean? = false
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
        val delayStartShoot: Double = 0.4
        val DelayBeforeShoot: Double = 0.1
        val delayAfterEachShoot: Double = 0.54 //currently at a really high #
        val DelayFromRampIntake: Double = 0.1
        val DelayInIntake: Double = 0.95
        //        val DelayAfterIntake: Double = 0.0
        val DelayAtLever: Double = 0.00

        val goalX = 3.0
        val goalY = 144.0 - 6.0

        val intakeSpeed = 0.5
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

    private var robotIntake1: PathChain? = null
    private var robotGoToShoot1: PathChain? = null //since preload also intakes, there is no intake1

    private var robotIntake2: PathChain? = null
    private var robotGoToShoot2: PathChain? = null

    private var robotOpenLeverFromFar: PathChain? = null
    private var robotOpenLeverFromClose: PathChain? = null

    private var robotBackupFromRamp: PathChain? = null
    private var LeverGoShoot: PathChain? = null

    private var roboCommonIntake: PathChain? = null
    private var roboCommonGoShoot: PathChain? = null

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
                BezierLine(
                    AutonPositions.Blue(AutonPositions.startPoseClose),
                    AutonPositions.Blue(AutonPositions.start24ShootPos)
                )
            )
//            .setTangentHeadingInterpolation()
//            .setLinearHeadingInterpolation(
//                AutonPositions.Blue(AutonPositions.startPoseClose).heading,
//                AutonPositions.Blue(AutonPositions.start24ShootPos).heading,
//                0.85
//            )
            .setConstantHeadingInterpolation(AutonPositions.Blue(AutonPositions.start24ShootPos).heading)
            .build()
        //1st Intake
        robotIntake1 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.start24ShootPos),
                    AutonPositions.Blue(AutonPositions.intake1Pos24)
                )
            )
//            .setTangentHeadingInterpolation()
            .addParametricCallback(0.95, IntakeCommand) //WHERE INTAKE COMMAND WILL NOW GO IG
            .setConstantHeadingInterpolation(AutonPositions.Blue(AutonPositions.intake1Pos24).heading)
            .build()
        //1st Go Shoot
        robotGoToShoot1 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.intake1Pos24),
                    AutonPositions.Blue(AutonPositions.start24ShootPos)
                )
            )
            .setConstantHeadingInterpolation(AutonPositions.Blue(AutonPositions.start24ShootPos).heading)
            .build()

        // Go to Gate
        robotOpenLeverFromClose = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    AutonPositions.Blue(AutonPositions.shootPoseClose),
                    AutonPositions.Blue(AutonPositions.gateOpenControlPos),
                    AutonPositions.Blue(AutonPositions.gateOpenPose),
                )
            )
            .setLinearHeadingInterpolation(
                AutonPositions.Blue(AutonPositions.shootPoseClose).heading,
                AutonPositions.Blue(AutonPositions.gateOpenPose).heading,
                0.85
            )
//            .setLinearHeadingInterpolation(
//                AutonPositions.Blue(AutonPositions.shootPoseClose).heading,
//                AutonPositions.Blue(AutonPositions.gateOpenPose).heading,
//                0.9,
//                0.7
//            )
            .build()

        // Go to Gate
        robotOpenLeverFromFar = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.shootPoseFar),
                    AutonPositions.Blue(AutonPositions.gateOpenPoseFromFar),
                )
            )
            .setLinearHeadingInterpolation(
                AutonPositions.Blue(AutonPositions.shootPoseFar).heading,
                AutonPositions.Blue(AutonPositions.gateOpenPoseFromFar).heading,
                0.9
            )
            .build()

        //backs up from lever to stay legal
        robotBackupFromRamp = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.gateOpenPose),
                    AutonPositions.Blue(AutonPositions.gateAfterOpenPose),
                )
            )
            .addParametricCallback(1.0, IntakeCommand) //WHERE INTAKE COMMAND WILL NOW GO IG
            .setLinearHeadingInterpolation(
                AutonPositions.Blue(AutonPositions.gateOpenPose).heading,
                AutonPositions.Blue(AutonPositions.gateAfterOpenPose).heading,
                0.75
            )
            .build()

        //Lever Go Shoot
        LeverGoShoot = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.gateAfterOpenPose),
                    AutonPositions.Blue(AutonPositions.shootPoseClose)
                )
            )
            .setLinearHeadingInterpolation(
                AutonPositions.Blue(AutonPositions.gateAfterOpenPose).heading,
                AutonPositions.Blue(AutonPositions.shootPoseClose).heading,
                0.9
            )
            .build()

        //2nd Intake
        robotIntake2 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.start24ShootPos),
                    AutonPositions.Blue(AutonPositions.intake2Pos24)
                )
            )
            .addParametricCallback(0.95, IntakeCommand) //WHERE INTAKE COMMAND WILL NOW GO IG
            .setConstantHeadingInterpolation(AutonPositions.Blue(AutonPositions.intake2Pos24).heading)
            .build()

        //2nd Go Shoot
        robotGoToShoot2 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.intake2Pos24),
                    AutonPositions.Blue(AutonPositions.shootPoseClose)
                )
            )
            .setLinearHeadingInterpolation(AutonPositions.Blue(AutonPositions.intake2Pos24).heading, AutonPositions.Blue(AutonPositions.shootPoseClose).heading, 0.5)
//            .setConstantHeadingInterpolation(AutonPositions.Blue(AutonPositions.shootPoseClose).heading)
            .build()

        //3rd Intake
        robotIntake3 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.shootPoseClose),
                    AutonPositions.Blue(AutonPositions.intake3Pos24)
                )
            )
            .addParametricCallback(0.95, IntakeCommand) //WHERE INTAKE COMMAND WILL NOW GO IG
            .setLinearHeadingInterpolation(
                AutonPositions.Blue(AutonPositions.shootPoseClose).heading,
                AutonPositions.Blue(AutonPositions.intake3Pos24).heading,
                0.65
            )
            .build()

        //3rd Go Shoot
        robotGoToShoot3 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.intake3Pos),
                    AutonPositions.Blue(AutonPositions.shootPoseClose)
                )
            )
            .setLinearHeadingInterpolation(
                AutonPositions.Blue(AutonPositions.intake3Pos).heading,
                AutonPositions.Blue(AutonPositions.shootPoseClose).heading,
                0.65
            )
            .build()
        //3rd Intake
        roboCommonIntake = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    AutonPositions.Blue(AutonPositions.shootPoseFar),
                    AutonPositions.Blue(AutonPositions.commonIntakeControlPos),
                    AutonPositions.Blue(AutonPositions.commonIntakePos)
                )
            )
            .setLinearHeadingInterpolation(
            AutonPositions.Blue(AutonPositions.shootPoseFar).heading,
            AutonPositions.Blue(AutonPositions.commonIntakePos).heading,
            0.75
            )
            .addParametricCallback(0.95, IntakeCommand) //WHERE INTAKE COMMAND WILL NOW GO IG
            .build()

        //3rd Go Shoot
        roboCommonGoShoot = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.commonIntakePos),
                    AutonPositions.Blue(AutonPositions.shootPoseClose)
                )
            )
            .setLinearHeadingInterpolation(
                AutonPositions.Blue(AutonPositions.commonIntakePos).heading,
                AutonPositions.Blue(AutonPositions.shootPoseClose).heading,
                0.75
            )
            .build()
        //Go Park
        robotPark = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.shootPoseFar),
                    AutonPositions.Blue(AutonPositions.autonParkPose)
                )
            )
            .setLinearHeadingInterpolation(
                AutonPositions.Blue(AutonPositions.shootPoseFar).heading,
                AutonPositions.Blue(AutonPositions.autonParkPose).heading,
                0.9
            )
            .build()
    }

    val intakePower: Command = InstantCommand {PedroComponent.follower.setMaxPower(intakeSpeed)}
    val maxPower: Command = InstantCommand {PedroComponent.follower.setMaxPower(1.0)}



    ///////////////////////////
    ////Main Auton Commands////
    ///////////////////////////
    val IntakeCommand: Command
        get() = ParallelGroup(
            intakePower,
            IntakeMotorSubsystem.intake,
            MagMotorSubsystem.intake,
//            MagServoSubsystem.run,
            MagblockServoSubsystem.block

        )
    val IntakeAfterCommand: Command
        get() = ParallelGroup(
            maxPower,
            IntakeMotorSubsystem.intake,
            MagMotorSubsystem.intake,
//            MagServoSubsystem.run,
            MagblockServoSubsystem.block

        )
    val TravelCommand: Command
        get() = ParallelGroup(
            maxPower,
            IntakeMotorSubsystem.off,
            MagMotorSubsystem.off,
//            MagServoSubsystem.stop,
            MagblockServoSubsystem.block,

        )
    val ShootCommand: Command
        get() = ParallelGroup(
            maxPower,
            MagblockServoSubsystem.unblock,
            MagMotorSubsystem.On(1.0),
            IntakeMotorSubsystem.intake,
//            MagServoSubsystem.run
        )

    /////////////////////////
    ////Autonomous Runner////
    /////////////////////////
    private val autonomousRoutine: Command
        get() = SequentialGroup(
            //Main Group
            SequentialGroup( //Shoots PRELOAD
                FollowPath(robotShootPreload!!),
                Delay(delayStartShoot),
                ShootCommand,
                Delay(delayAfterEachShoot),
                TravelCommand,
//                IntakeCommand,
                FollowPath(robotIntake1!!),
            ),

            ParallelGroup( //Robot goes back to FAR Shoot Position
//                TurretPhiSubsystem.SetTargetPhi((- (-5.075 + 2.0 * PI - PI / 3.0 - PI / 48.0)).rad),
                SequentialGroup(
                    IntakeAfterCommand,
                    Delay(DelayInIntake),
                    TravelCommand,
                ),
                FollowPath(robotGoToShoot1!!)
            ),

            SequentialGroup( //Shoots FIRST Intake, goes to intake
                Delay(DelayBeforeShoot),
                ShootCommand,
                Delay(delayAfterEachShoot),
                TravelCommand,
//                TravelCommand,
//                ParallelGroup(
//                    FollowPath(robotIntake2!!), //robot goes to intake
//                    SequentialGroup(
////                        Delay(PedroComponent.follower.currentPath.),
//                        IntakeCommand
//                    )
//                ),
//                IntakeCommand,
                FollowPath(robotIntake2!!), //robot goes to intake
//                Delay(99999.9),
            ),

            ParallelGroup( //Robot goes back to FAR Shoot Position
                SequentialGroup(
                    IntakeAfterCommand,
                    Delay(DelayInIntake),
                    TravelCommand,
                ),
                FollowPath(robotGoToShoot2!!)
            ),

            //////////////////Lever Portion//////////////////
            //////////////////Lever Portion//////////////////
            //////////////////Lever Portion//////////////////
            SequentialGroup( //Shoots SECOND Intake, goes to intake from the lever
                Delay(DelayBeforeShoot),
                ShootCommand,
                Delay(delayAfterEachShoot),
                TravelCommand,
                FollowPath(robotOpenLeverFromClose!!), //robot goes to intake
                Delay(DelayAtLever),
                FollowPath(robotBackupFromRamp!!),
                Delay(DelayFromRampIntake),
            ),

            ParallelGroup( //Robot goes back to FAR Shoot Position
                SequentialGroup(
                    IntakeAfterCommand,
                    Delay(DelayInIntake),
                    TravelCommand,
                ),
                FollowPath(LeverGoShoot!!)
            ),

            SequentialGroup( //Shoots SECOND Intake, goes to intake from the lever
                Delay(DelayBeforeShoot),
                ShootCommand,
                Delay(delayAfterEachShoot),
                TravelCommand,
                FollowPath(robotOpenLeverFromClose!!), //robot goes to intake
                Delay(DelayAtLever),
                FollowPath(robotBackupFromRamp!!),
                Delay(DelayFromRampIntake),
            ),

            ParallelGroup( //Robot goes back to FAR Shoot Position
                SequentialGroup(
                    IntakeAfterCommand,
                    Delay(DelayInIntake),
                    TravelCommand,
                ),
                FollowPath(LeverGoShoot!!)
            ),
            //////////////////Lever Portion//////////////////
            //////////////////Lever Portion//////////////////
            //////////////////Lever Portion//////////////////

            SequentialGroup( //Shoots THIRD Intake, goes to intake
                Delay(DelayBeforeShoot),
                ShootCommand,
                Delay(delayAfterEachShoot),
                TravelCommand,
                FollowPath(robotIntake3!!), //robot goes to intake
//                Delay(DelayAfterIntake),
            ),

            ParallelGroup( //Robot goes back to FAR Shoot Position
                SequentialGroup(
                    IntakeAfterCommand,
                    Delay(DelayInIntake),
                    TravelCommand,
                ),
                FollowPath(robotGoToShoot3!!)
            ),

            //////////////////Lever Portion//////////////////
            //////////////////Lever Portion//////////////////
            //////////////////Lever Portion//////////////////
//            SequentialGroup( //Shoots FOURTH Intake, goes to intake from the lever
//                Delay(DelayBeforeShoot),
//                ShootCommand,
//                Delay(delayAfterEachShoot),
//                IntakeCommand,
//                FollowPath(robotOpenLeverFromFar!!), //robot goes to intake
//                Delay(DelayAtLever),
//                FollowPath(robotBackupFromRamp!!),
//                Delay(DelayFromRampIntake),
//            ),
//
//            ParallelGroup( //Robot goes back to FAR Shoot Position
//                SequentialGroup(
//                    Delay(DelayInIntake),
//                    TravelCommand,
//                ),
//                FollowPath(LeverGoShoot!!)
//            ),
            //////////////////Lever Portion//////////////////
            //////////////////Lever Portion//////////////////
            //////////////////Lever Portion//////////////////

            ///////////////////////////////////////////
            ///////////////COMMON INTAKE///////////////
            ///////////////////////////////////////////
            SequentialGroup( //Shoots FIFTH Intake, goes to intake
                Delay(DelayBeforeShoot),
                ShootCommand,
                Delay(delayAfterEachShoot),
                TravelCommand,
                FollowPath(roboCommonIntake!!), //robot goes to intake
//                Delay(DelayAfterIntake),
            ),

            ParallelGroup( //Robot goes back to FAR Shoot Position
                SequentialGroup(
                    IntakeAfterCommand,
                    Delay(DelayInIntake),
                    TravelCommand,
                ),
                FollowPath(roboCommonGoShoot!!)
            ),
            ///////////////////////////////////////////
            ///////////////COMMON INTAKE///////////////
            ///////////////////////////////////////////

            SequentialGroup( //Shoots SEVENTH Intake, goes to PARK
                Delay(DelayBeforeShoot),
                ShootCommand,
                Delay(delayAfterEachShoot),
                TravelCommand,
                InstantCommand { stopShooterAutoAim = true },
                ShooterSubsystem.On(9999.0),
                FollowPath(robotPark!!), //robot goes to intake
            )
        )

    private var stopShooterAutoAim = false;
    override fun onUpdate() {
//        if (pathStarted!! && opmodeTimer!!.elapsedTime >= 29.5) {
//            PedroComponent.follower.setMaxPower(0.0)
//            PedroComponent.follower.breakFollowing()
//            IntakeMotorSubsystem.off
//            MagMotorSubsystem.off
//            MagblockServoSubsystem.block
//        }
//        val dxp = dx - (PedroComponent.follower.velocity.xComponent
//            + 0.05 * PedroComponent.follower.acceleration.xComponent) * (
//                if (PedroComponent.follower.pose.y < BORD_Y) distanceToTimeFar(dxy)
//                else distanceToTimeClose(dxy)
//        )
//        val dyp = dy - (PedroComponent.follower.velocity.yComponent
//            + 0.05 * PedroComponent.follower.acceleration.yComponent) * (
//                if (PedroComponent.follower.pose.y < BORD_Y) distanceToTimeFar(dxy)
//                else distanceToTimeClose(dxy)
//                )

        val dx = goalX - PedroComponent.follower.pose.x
        val dy = goalY - PedroComponent.follower.pose.y
        val dxy = hypot(dx, dy)
        val dxp = dx;
        val dyp = dy;
        val dxyp = hypot(dxp, dyp)
        if (!stopShooterAutoAim) {
            ShooterSubsystem.AutoAim(
                dxyp,
                { dist ->
                    (
                            if (PedroComponent.follower.pose.y < BORD_Y)
                                distanceToVelocityFar(dist)
                            else
                                distanceToVelocityClose(dist)
                            )
                }
            )()
        }
        TurretPhiSubsystem.AutoAim(dxp, dyp, PedroComponent.follower.pose.heading.rad)();
        TurretThetaSubsystem.AutoAim(
            dxyp,
            { dist ->
                if (PedroComponent.follower.pose.y < BORD_Y)
                    distAndVeloToThetaFar(dist, ShooterSubsystem.velocity)
                else
                    distAndVeloToThetaClose(dist, ShooterSubsystem.velocity)
            },
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
//        MagServoSubsystem.stop()
        MagblockServoSubsystem.block()

        PedroComponent.follower.setStartingPose(AutonPositions.Blue(AutonPositions.startPoseClose))
    }

    /** This method is called continuously after Init while waiting for "play".  */
    override fun onWaitForStart() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system  */
    override fun onStartButtonPressed() {
        autonomousRoutine()
        pathStarted = true

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