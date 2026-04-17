package org.firstinspires.ftc.teamcode.Auton.MainAutons

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.HeadingInterpolator
import com.pedropathing.paths.PathChain
import com.pedropathing.util.Timer
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.Auton.AutonPositions
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.LowerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagblockServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
import org.firstinspires.ftc.teamcode.utils.Lefile
import java.io.File

@Autonomous(name = "[ARTI-24-B-STA-0] Auton Blue Close Artifact", group = "Auton")
@Configurable
class AutonBlueCloseArtifact24: NextFTCOpMode() {
    //////////////////////
    ////Base Variables////
    //////////////////////
    private var pathTimer: Timer? = null
    var actionTimer: Timer? = null
    var opmodeTimer: Timer? = null
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
        val DelayBeforeShoot: Double = 0.0
        val delayAfterEachShoot: Double = 0.27 //currently at a really high #
        val DelayFromRampIntake: Double = 0.4
        val DelayInIntake: Double = 0.65
        //        val DelayAfterIntake: Double = 0.0
        val DelayAtLever: Double = 0.00

        var canShoot: Boolean = false

        val goalX = 1.0
        val goalY = 144.0 - 1.0
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

    private var roboSPEDIntake: PathChain? = null
    private var roboSPEDGoShoot: PathChain? = null

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
                    AutonPositions.Blue(AutonPositions.shootPoseClose)
                )
            )
//            .setTangentHeadingInterpolation()
            .setLinearHeadingInterpolation(
                AutonPositions.Blue(AutonPositions.startPoseClose).heading,
                AutonPositions.Blue(AutonPositions.shootPoseClose).heading,
                0.85
            )
//            .setConstantHeadingInterpolation(AutonPositions.Blue(AutonPositions.start24ShootPos).heading)
            .build()
        //1st Intake
        robotIntake1 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    AutonPositions.Blue(AutonPositions.shootPoseCloseAlt),
                    AutonPositions.Blue(AutonPositions.intake1Pos24CPos),
                    AutonPositions.Blue(AutonPositions.intake1Pos24)
                )
            )
            .setTangentHeadingInterpolation()
            .addParametricCallback(0.22, IntakeCommand) //WHERE INTAKE COMMAND WILL NOW GO IG
//            .setConstantHeadingInterpolation(AutonPositions.Blue(AutonPositions.intake1Pos24).heading)
            .build()
        //1st Go Shoot
        robotGoToShoot1 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.intake1Pos24),
                    AutonPositions.Blue(AutonPositions.shootPoseCloseAlt)
                )
            )
            .setHeadingInterpolation(
                HeadingInterpolator.tangent.reverse()
            )
//            .setConstantHeadingInterpolation(AutonPositions.Blue(AutonPositions.start24ShootPos).heading)
            .build()

        // Go to Gate
        robotOpenLeverFromClose = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    AutonPositions.Blue(AutonPositions.shootPoseCloseAlt),
                    AutonPositions.Blue(AutonPositions.gateOpenControlPos),
                    AutonPositions.Blue(AutonPositions.gateOpenPose)
                )
            )
//            .addParametricCallback(0.75, intakePower)
//            .addParametricCallback(0.98, maxPower)
//            .setTangentHeadingInterpolation()
            .setLinearHeadingInterpolation(
                AutonPositions.Blue(AutonPositions.shootPoseCloseAlt).heading,
                AutonPositions.Blue(AutonPositions.gateOpenPose).heading,
                0.6
            )
            .build()

        //backs up from lever to stay legal  // todo : nathan note, I think we can cut the backup
        robotBackupFromRamp = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.gateOpenPose),
//                    AutonPositions.Blue(AutonPositions.gateToShootControlPos),
                    AutonPositions.Blue(AutonPositions.gateAfterOpenPose),
                )
            )
            .addParametricCallback(0.05, IntakeAfterCommand)
            .addParametricCallback(0.75, IntakeCommand)
//            .setHeadingInterpolation(
//                HeadingInterpolator.tangent.reverse()
//            )
            .setConstantHeadingInterpolation(AutonPositions.Blue(AutonPositions.gateAfterOpenPose).heading)
            .build()

        //Lever Go Shoot
        LeverGoShoot = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    AutonPositions.Blue(AutonPositions.gateAfterOpenPose),
                    AutonPositions.Blue(AutonPositions.gateToShootControlPos),
                    AutonPositions.Blue(AutonPositions.shootPoseCloseAlt)
                )
            )
            .setHeadingInterpolation(
                HeadingInterpolator.tangent.reverse()
            )
            .setTimeoutConstraint(0.0)
            .build()

        //2nd Intake
        robotIntake2 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    AutonPositions.Blue(AutonPositions.shootPoseClose),
                    AutonPositions.Blue(AutonPositions.intake2P24CPos),
                    AutonPositions.Blue(AutonPositions.intake2Pos24)
                )
            )
            .addParametricCallback(0.7, IntakeCommand) //WHERE INTAKE COMMAND WILL NOW GO IG
            .setTangentHeadingInterpolation()
//            .setConstantHeadingInterpolation(AutonPositions.Blue(AutonPositions.intake2Pos24).heading)
            .build()

        //2nd Go Shoot
        robotGoToShoot2 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.intake2Pos24),
                    AutonPositions.Blue(AutonPositions.shootPoseCloseAlt)
                )
            )
            .setLinearHeadingInterpolation(AutonPositions.Blue(
                AutonPositions.intake2Pos24).heading,
                AutonPositions.Blue(AutonPositions.shootPoseCloseAlt).heading,
                0.5)
//            .setConstantHeadingInterpolation(AutonPositions.Blue(AutonPositions.shootPoseCloseAlt).heading)
            .setTimeoutConstraint(0.0)
            .build()

        //3rd Intake
        robotIntake3 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.shootPoseCloseAlt),
                    AutonPositions.Blue(AutonPositions.intake3Pos24)
                )
            )
            .addParametricCallback(0.75, IntakeCommand)
            .setLinearHeadingInterpolation(
                AutonPositions.Blue(AutonPositions.shootPoseCloseAlt).heading,
                AutonPositions.Blue(AutonPositions.intake3Pos24).heading,
                0.65
            )
            .build()

        //3rd Go Shoot
        robotGoToShoot3 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.intake3Pos24),
                    AutonPositions.Blue(AutonPositions.shootPoseCloseAlt)
                )
            )
            .setLinearHeadingInterpolation(
                AutonPositions.Blue(AutonPositions.intake3Pos24).heading,
                AutonPositions.Blue(AutonPositions.shootPoseCloseAlt).heading,
                0.65
            )
            .setTimeoutConstraint(0.0)
            .build()
        //3rd Intake
        roboCommonIntake = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    AutonPositions.Blue(AutonPositions.shootPoseCloseAlt),
                    AutonPositions.Blue(AutonPositions.commonIntakeControlPos24),
                    AutonPositions.Blue(AutonPositions.commonIntakePos24)
                )
            )
            .setTangentHeadingInterpolation()
//            .setLinearHeadingInterpolation(
//            AutonPositions.Blue(AutonPositions.shootPoseClose).heading,
//            AutonPositions.Blue(AutonPositions.commonIntakePos24).heading,
//            0.75
//            )
            .addParametricCallback(0.95, IntakeCommand)
            .build()

        //3rd Go Shoot
        roboCommonGoShoot = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.commonIntakePos),
                    AutonPositions.Blue(AutonPositions.shootPoseCloseAlt)
                )
            )
            .setHeadingInterpolation(
                HeadingInterpolator.tangent.reverse()
            )
            .setTimeoutConstraint(0.0)
//            .setLinearHeadingInterpolation(
//                AutonPositions.Blue(AutonPositions.commonIntakePos).heading,
//                AutonPositions.Blue(AutonPositions.shootPoseClose).heading,
//                0.75
//            )
            .build()
        roboSPEDIntake = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    AutonPositions.Blue(AutonPositions.shootPoseCloseAlt),
                    AutonPositions.Blue(AutonPositions.spedPosition24ControlPos),
                    AutonPositions.Blue(AutonPositions.spedPosition24ControlPos2),
                    AutonPositions.Blue(AutonPositions.spedPosition24)
                )
            )
            .setTangentHeadingInterpolation()
//            .setLinearHeadingInterpolation(
//                AutonPositions.Blue(AutonPositions.shootPoseFar).heading,
//                AutonPositions.Blue(AutonPositions.commonIntakePos24).heading,
//                0.75
//            )
            .addParametricCallback(0.3, IntakeAfterCommand)
            .build()

        //3rd Go Shoot
        roboSPEDGoShoot = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.commonIntakePos),
                    AutonPositions.Blue(AutonPositions.shootPoseClose)
                )
            )
            .setHeadingInterpolation(
                HeadingInterpolator.tangent.reverse()
            )
//            .setLinearHeadingInterpolation(
//                AutonPositions.Blue(AutonPositions.commonIntakePos).heading,
//                AutonPositions.Blue(AutonPositions.shootPoseClose).heading,
//                0.75
//            )
            .setTimeoutConstraint(0.0)
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
            .setTimeoutConstraint(0.0)
            .build()
    }

    ///////////////////////////
    ////Main Auton Commands////
    ///////////////////////////
    val intakePower: Command = InstantCommand {PedroComponent.follower.setMaxPower(0.75)}
    val maxPower: Command = InstantCommand {PedroComponent.follower.setMaxPower(1.0)}

    val SetCanShootFalse: Command = InstantCommand {canShoot = false}
    val SetCanShootTrue: Command = InstantCommand {canShoot = true}

    val stopFollower: Command = InstantCommand {PedroComponent.follower.breakFollowing()}

    val IntakeCommand: Command
        get() = ParallelGroup(
            SetCanShootTrue,
            intakePower,
            IntakeMotorSubsystem.intake,
            MagMotorSubsystem.intake,
//            MagServoSubsystem.run,
            MagblockServoSubsystem.block

        )
    val IntakeAfterCommand: Command
        get() = ParallelGroup(
            SetCanShootTrue,
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
            SetCanShootFalse,
            maxPower,
            MagblockServoSubsystem.unblock,
            MagMotorSubsystem.On(1.0),
            IntakeMotorSubsystem.intake,
//            MagServoSubsystem.run
        )

    fun robotShoot(): Command {
        return SequentialGroup(
            Delay(DelayBeforeShoot),
            ShootCommand,
            Delay(delayAfterEachShoot),
            stopFollower,
            TravelCommand,
        )
    }

    fun robotIntake(followedPath: PathChain?): Command {
        return SequentialGroup(
            TravelCommand,
            FollowPath(followedPath!!)
        )
    }

    fun robotGateIntake(followedPath: PathChain?, goBackPath: PathChain?): Command {
        return SequentialGroup(
            TravelCommand,
            FollowPath(followedPath!!), //robot goes to intake
            Delay(DelayAtLever),
            FollowPath(goBackPath!!),
            Delay(DelayFromRampIntake),
        )
    }

    fun robotGoShoot(followedPath: PathChain?): Command {
        return ParallelGroup(
            SequentialGroup(
                IntakeAfterCommand,
                Delay(DelayInIntake),
                TravelCommand,
            ),
            FollowPath(followedPath!!, true)
        )
    }

    //    double[] x_top = {72,-8,152};
    //    double[] y_top = {64,144,144};
    //    double[] x_bottom = {72,40,104};
    //    double[] y_bottom = {32,0,0};

    fun inTriangle(x1: Double, y1: Double): Int {
        val MARGIN = 8
        /**0 = none, 1 = top, 2 = bottom, 3 = error */
        if (x1 > 144 || x1 < 0 || y1 > 144 || y1 < 0) {
            return 3
        }
        // T triangle: vertices (72,64), (-8,144), (152,144)
        val inTop = (y1 <= -x1 + 144 - MARGIN) && (y1 <= x1 - MARGIN)

        // B triangle:  (40,0), (72,32), (104,0)
        val inBottom = (y1 <= x1 - (48 - MARGIN)) && (y1 <= -x1 + 96 + MARGIN)

        if (inTop) return 1
        if (inBottom) return 2
        return 0
    }

    fun parkRobot(): Command {
        return SequentialGroup(
            InstantCommand { stopShooterAutoAim = true },
            ShooterSubsystem.On(9999.0),
            FollowPath(robotPark!!)
        )
    }

    /////////////////////////
    ////Autonomous Runner////
    /////////////////////////
    private val autonomousRoutine: Command
        get() = SequentialGroup(
            ParallelGroup(
                //Shoots PRELOAD which had [3 ARTIFACTS]
                ShooterSubsystem.On(1500.0),
                FollowPath(robotShootPreload!!),
                SequentialGroup(
                    Delay(delayStartShoot),
                    SetCanShootTrue,
//                    robotShoot(),
                ),
            ),

            //Intakes to [6 ARTIFACTS] then shoots
            robotIntake(robotIntake2),
            robotGoShoot(robotGoToShoot2),
//            robotShoot(),

            //Gate intakes to [9 ARTIFACTS] then shoots
            robotGateIntake(robotOpenLeverFromClose, robotBackupFromRamp),
            robotGoShoot(LeverGoShoot),
//            robotShoot(),

            //Intakes to [12 ARTIFACTS] then shoots
            robotIntake(robotIntake1),
            robotGoShoot(robotGoToShoot1),
//            robotShoot(),

            //Gate intakes to [15 ARTIFACTS] then shoots
            robotGateIntake(robotOpenLeverFromClose, robotBackupFromRamp),
            robotGoShoot(LeverGoShoot),
//            robotShoot(),

            //Intakes to [18 ARTIFACTS] then shoots
            robotIntake(robotIntake3),
            robotGoShoot(robotGoToShoot3),
//            robotShoot(),

            //Gate intakes to [21 ARTIFACTS] then shoots
            robotGateIntake(robotOpenLeverFromClose, robotBackupFromRamp),
            robotGoShoot(LeverGoShoot),
//            robotShoot(),

            //Intakes to [24 ARTIFACTS] then shoots
            robotIntake(roboCommonIntake),
            robotGoShoot(roboCommonGoShoot),
//            robotShoot(),

//            //Intakes to [27 ARTIFACTS] then shoots
////            robotShoot(),
////            robotIntake(roboSPEDIntake),
////            robotGoShoot(roboSPEDGoShoot),
//
//            //Shoots last intake and then parks [RP Points + 3 points]
//            robotShoot(),
//            parkRobot()
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

//        val dx = goalX - PedroComponent.follower.pose.x
//        val dy = goalY - PedroComponent.follower.pose.y
//        val dxy = hypot(dx, dy)
//        val dxp = dx;
//        val dyp = dy;
//        val dxyp = hypot(dxp, dyp)
//        if (!stopShooterAutoAim) {
//            ShooterSubsystem.AutoAim(
//                dxyp,
//                { dist ->
//                    (
//                            if (PedroComponent.follower.pose.y < BORD_Y)
//                                distanceToVelocityFar(dist)
//                            else
//                                distanceToVelocityClose(dist)
//                            )
//                }
//            )()
//        }
//        TurretPhiSubsystem.AutoAim(dxp, dyp, PedroComponent.follower.pose.heading.rad)();
//        TurretThetaSubsystem.AutoAim(
//            dxyp,
//            { dist ->
//                if (PedroComponent.follower.pose.y < BORD_Y)
//                    distAndVeloToThetaFar(dist, ShooterSubsystem.velocity)
//                else
//                    distAndVeloToThetaClose(dist, ShooterSubsystem.velocity)
//            },
//        )()

        // These loop the movements of the robot, these must be called continuously in order to work
//        follower!!.update();
//        forward.schedule()
        // Feedback to Driver Hub for debugging
//        telemetry.addData("path state", pathState)

        telemetry.addData("canShoot", canShoot)
        telemetry.addData("InTriangle", inTriangle(PedroComponent.follower.pose.x, PedroComponent.follower.pose.y))
        telemetry.addData("x", PedroComponent.follower.pose.x)
        telemetry.addData("y", PedroComponent.follower.pose.y)
        telemetry.addData("heading", PedroComponent.follower.pose.heading)
        telemetry.update()

        PanelsTelemetry.telemetry.update()

        if (canShoot && inTriangle(PedroComponent.follower.pose.x, PedroComponent.follower.pose.y) == 1) {
            robotShoot()
        }
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