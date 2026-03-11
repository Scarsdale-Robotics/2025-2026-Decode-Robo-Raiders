package org.firstinspires.ftc.teamcode.Auton.MainAutons

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
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
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.Auton.AutonPositions
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.LowerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem

import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagServoSubsystem
//import org.firstinspires.ftc.teamcode.subsystems.lower.LowerMotorSubsystem

import org.firstinspires.ftc.teamcode.subsystems.lower.MagblockServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.BORD_Y
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.distAndVeloToThetaClose
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.distAndVeloToThetaFar
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.distanceToVelocityClose
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.distanceToVelocityFar
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
@Autonomous(name = "[PUSH-15] Auton Blue Far Push", group = "Auton")
@Configurable
class AutonBlueFarPush: NextFTCOpMode(){ //Pretend robot is 14 to 16 (14 is intake to backplate)
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
        val delayStartup = 2.0;
        val delayFarShoot = 0.825;
        val delayAtGate = 0.01;
        val delayPreShoot = 0.8;
        val delayCloseShoot = 0.41;
        val delayAfterIntake = 0.0;
        val delayAfterGate = 0.75
        val delayInIntake = 0.4;

        val goalX = 2.5
        val goalY = 144.0 - 3.5
        var pathStarted = false;
//        var directionGoalX = 4.0;
//        var directionGoalY = 144.0-4.0;
    }


    /////////////////
    ////Positions////
    /////////////////
    //Constant positions

    /////////////
    ////Paths////
    /////////////
    // The different paths the robot will take in during Auton
    private var pushPath: PathChain? = null

    private var pushToClose: PathChain? = null;

    private var closeToL2: PathChain? = null
    private var L2Shoot: PathChain? = null

    private var closeToGate: PathChain? = null
    private var gateToAfter: PathChain? = null

    private var gateToShoot: PathChain? = null

    private var closeToL1: PathChain? = null
    private var L1Shoot: PathChain? = null

    private var closeToL3: PathChain? = null
    private var L3Shoot: PathChain? = null

    private var closeToHPZ: PathChain? = null
    private var HPZShoot: PathChain? = null

    private var parkPath: PathChain? = null

//    private val LOW_BRAKING = 0.6;
//    private val HIGH_BRAKING = 0.8;

    private val LOW_BRAKING = 0.75;
    private val HIGH_BRAKING = 0.75;

    ////////////////////
    ////Path Builder////
    ////////////////////
    fun buildPaths() {
        //1st Intake
        pushPath = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.startPoseFarPush),
                    AutonPositions.Blue(AutonPositions.postPushPose),
                )
            )
            .setBrakingStrength(HIGH_BRAKING)
            .setConstantHeadingInterpolation(AutonPositions.Blue(Math.toRadians(270.0)))
            .build()
        pushToClose = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    AutonPositions.Blue(AutonPositions.postPushPose),
                    AutonPositions.Blue(AutonPositions.shootPoseClose),
                )
            )
            .setBrakingStrength(LOW_BRAKING)
//            .addParametricCallback(0.75, intakePower)
//            .addParametricCallback(0.98, maxPower)
//            .setTimeoutConstraint(delayPreShoot / 2.0)
            .setConstantHeadingInterpolation(AutonPositions.Blue(Math.toRadians(270.0)))
            .build()
        closeToL2 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    AutonPositions.Blue(AutonPositions.shootPoseClose),
                    AutonPositions.Blue(AutonPositions.L2IntakeControlPose),
                    AutonPositions.Blue(AutonPositions.L2IntakePose),
                )
            )
            .setBrakingStrength(HIGH_BRAKING)
////            .setTimeoutConstraint(delayAfterIntake / 2.0)
            .addParametricCallback(0.64, IntakeCommand)
            .addParametricCallback(0.69, IntakeAfterCommand)
            .setConstantHeadingInterpolation(AutonPositions.Blue(Math.toRadians(180.0)))
            .build()
        L2Shoot = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.L2IntakePose),
                    AutonPositions.Blue(AutonPositions.shootPoseClose)
                )
            )
            .setBrakingStrength(LOW_BRAKING)
//            .addParametricCallback(0.75, intakePower)
//            .addParametricCallback(0.98, maxPower)
//            .setTimeoutConstraint(delayPreShoot / 2.0)
//            .addParametricCallback(0.0, maxPower)
            .setConstantHeadingInterpolation(AutonPositions.Blue(Math.toRadians(215.0)))
            .build()
        closeToGate = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    AutonPositions.Blue(AutonPositions.shootPoseClose),
                    AutonPositions.Blue(AutonPositions.gateOpenControlPos),
                    AutonPositions.Blue(AutonPositions.gateOpenPose)
                )
            )
            .setBrakingStrength(HIGH_BRAKING)
            .addParametricCallback(0.75, intakePower)
            .addParametricCallback(0.98, maxPower)
            .setLinearHeadingInterpolation(
                AutonPositions.Blue(AutonPositions.shootPoseClose).heading,
                AutonPositions.Blue(AutonPositions.gateOpenPose).heading,
                0.9
            )
            .build()
        gateToAfter = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.gateOpenPose),
//                    AutonPositions.Blue(AutonPositions.gateToShootControlPos),
                    AutonPositions.Blue(AutonPositions.gateAfterOpenPose),
                )
            )
            .setBrakingStrength(HIGH_BRAKING)
            .addParametricCallback(0.05, IntakeAfterCommand)
            .addParametricCallback(0.75, IntakeCommand)
            .setConstantHeadingInterpolation(AutonPositions.Blue(AutonPositions.gateAfterOpenPose).heading)
            .build()
        gateToShoot = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    AutonPositions.Blue(AutonPositions.gateAfterOpenPose),
                    AutonPositions.Blue(AutonPositions.gateToShootControlPos),
                    AutonPositions.Blue(AutonPositions.shootPoseClose)
                )
            )
            .setBrakingStrength(HIGH_BRAKING)
//            .addParametricCallback(0.75, intakePower)
//            .addParametricCallback(0.98, maxPower)
//            .setTimeoutConstraint(delayPreShoot / 2.0)
            .setHeadingInterpolation(
                HeadingInterpolator.tangent.reverse()
            )
            .build()
        closeToL1 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    AutonPositions.Blue(AutonPositions.shootPoseClose),
                    AutonPositions.Blue(AutonPositions.L1IntakeControlPose),
                    AutonPositions.Blue(AutonPositions.L1IntakePose),
                )
            )
            .setBrakingStrength(HIGH_BRAKING)
////            .setTimeoutConstraint(delayAfterIntake / 2.0)
            .addParametricCallback(0.49, IntakeCommand)
            .addParametricCallback(0.535, IntakeAfterCommand)
            .setTangentHeadingInterpolation()
            .build()
        L1Shoot = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.L1IntakePose),
                    AutonPositions.Blue(AutonPositions.shootPoseClose),
                )
            )
            .setBrakingStrength(LOW_BRAKING)
//            .setTimeoutConstraint(delayPreShoot / 2.0)
//            .addParametricCallback(0.0, maxPower)
            .setLinearHeadingInterpolation(
                AutonPositions.Blue(AutonPositions.L1IntakePose).heading,
                AutonPositions.Blue(AutonPositions.shootPoseClose).heading,
            )
//            .addParametricCallback(0.75, intakePower)
//            .addParametricCallback(0.98, maxPower)
            .setConstantHeadingInterpolation(AutonPositions.Blue(Math.toRadians(215.0)))
            .build()
        closeToL3 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    AutonPositions.Blue(AutonPositions.shootPoseClose),
                    AutonPositions.Blue(AutonPositions.L3IntakeControlPose),
                    AutonPositions.Blue(AutonPositions.L3IntakePose),
                )
            )
            .setBrakingStrength(HIGH_BRAKING)
////            .setTimeoutConstraint(delayAfterIntake / 2.0)
            .addParametricCallback(0.59, IntakeCommand)
            .addParametricCallback(0.64, IntakeAfterCommand)
            .setTangentHeadingInterpolation()
            .build()
        L3Shoot = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.L3IntakePose),
                    AutonPositions.Blue(AutonPositions.shootPoseClose),
                )
            )
            .setBrakingStrength(LOW_BRAKING)
//            .addParametricCallback(0.85, intakePower)
//            .addParametricCallback(0.98, maxPower)
//            .setTimeoutConstraint(delayPreShoot / 2.0)
//            .addParametricCallback(0.0, maxPower)
            .setHeadingInterpolation(
                HeadingInterpolator.tangent.reverse()
            )
            .build()
        closeToHPZ = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    AutonPositions.Blue(AutonPositions.shootPoseClose),
                    AutonPositions.Blue(AutonPositions.HPZControlPose),
                    AutonPositions.Blue(AutonPositions.HPZPose),
                )
            )
            .setBrakingStrength(HIGH_BRAKING)
////            .setTimeoutConstraint(delayAfterIntake / 2.0)
            .addParametricCallback(0.86, IntakeCommand)
            .setTangentHeadingInterpolation()
            .build()
        HPZShoot = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    AutonPositions.Blue(AutonPositions.HPZPose),
                    AutonPositions.Blue(AutonPositions.HPZControlPose),
                    AutonPositions.Blue(AutonPositions.shootPoseClose),
                )
            )
            .setBrakingStrength(LOW_BRAKING)
//            .addParametricCallback(0.75, intakePower)
//            .addParametricCallback(0.98, maxPower)
//            .setTimeoutConstraint(delayPreShoot / 2.0)
            .setHeadingInterpolation(
                HeadingInterpolator.tangent.reverse()
            )
            .build()
        parkPath = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.shootPoseClose),
                    AutonPositions.Blue(AutonPositions.pApark),
                )
            )
            .setBrakingStrength(HIGH_BRAKING)
            .setTimeoutConstraint(0.0)
            .setHeadingInterpolation(
                HeadingInterpolator.tangent.reverse()
            )
//            .setTangentHeadingInterpolation()
            .build()
    }

    val intakePower: Command = InstantCommand {PedroComponent.follower.setMaxPower(0.6)}
    val maxPower: Command = InstantCommand {PedroComponent.follower.setMaxPower(1.0)}

    val IntakeCommand: Command
        get() = ParallelGroup(
            intakePower,
            IntakeMotorSubsystem.intake,
            MagMotorSubsystem.intake,
            MagServoSubsystem.run,
            MagblockServoSubsystem.block
        )
    val IntakeAfterCommand: Command
        get() = ParallelGroup(
            maxPower,
            IntakeMotorSubsystem.intake,
            MagMotorSubsystem.intake,
            MagServoSubsystem.run,
            MagblockServoSubsystem.block
        )
    val TravelCommand: Command
        get() = ParallelGroup(
            maxPower,
            IntakeMotorSubsystem.off,
            MagMotorSubsystem.off,
            MagServoSubsystem.stop,
            MagblockServoSubsystem.block
        )
    val ShootCommandClose: Command
        get() = ParallelGroup(
            maxPower,
            MagblockServoSubsystem.unblock,
            MagMotorSubsystem.On(1.0),
            IntakeMotorSubsystem.intake,
            MagServoSubsystem.run
        )
    val ShootCommandFar: Command
        get() = ParallelGroup(
            maxPower,
            MagblockServoSubsystem.unblock,
            MagMotorSubsystem.On(0.85),
            IntakeMotorSubsystem.intake,
            MagServoSubsystem.run
        )

    var sotmFactor = 1.0;
    val EnableSOTM: Command = InstantCommand { sotmFactor = 1.0 }
    val DisableSOTM: Command = InstantCommand { sotmFactor = 0.0 }

//    val Intake = { path: PathChain ->
//        SequentialGroup(
//            FollowPath(path),  // intake L2
//            Delay(delayAfterIntake),
//        )
//    }
//    val RawApproachShoot = { path: PathChain ->
//        SequentialGroup(
//            EnableSOTM,
//            FollowPath(path),
//            DisableSOTM
//        )
//    }
//    val ShootClose = SequentialGroup(
//            Delay(delayPreShoot),
//            ShootCommandClose,
//            Delay(delayCloseShoot),
//        )
//    val ShootFar = SequentialGroup(
//            Delay(delayPreShoot),
//            ShootCommandFar,
//            Delay(delayFarShoot),
//        )
//    val PostIntake = SequentialGroup(
//            IntakeAfterCommand,
//            Delay(delayInIntake),
//            TravelCommand,
//        )
//    val PostIntakeApproachShoot = { shootPath: PathChain ->
//        ParallelGroup(
//            PostIntake,
//            RawApproachShoot(shootPath)
//        )
//    }
//    val IntakeApproachShoot = { intakePath: PathChain, shootPath: PathChain ->
//        SequentialGroup(
//            TravelCommand,
//            Intake(intakePath),
//            Delay(delayAfterIntake),
//            PostIntakeApproachShoot(shootPath)
//        )
//    }
//    val GateIntakeClose = SequentialGroup(
//        FollowPath(closeToGate!!),
//        Delay(delayAtGate),
//        FollowPath(gateToAfter!!),
//        Delay(delayAfterGate),
//    )
//    val Park = SequentialGroup(
//        TravelCommand,
//        InstantCommand { stopShooterAutoAim = true },
//        ShooterSubsystem.On(9999.0),
//        FollowPath(parkPath!!),
//    )

    /////////////////////////
    ////Autonomous Runner////
    /////////////////////////
    var stopShooterAutoAim = false;
//    private val autonomousRoutine = SequentialGroup(
//        FollowPath(pushPath!!),
//        RawApproachShoot(pushToClose!!),
//        ShootClose,
//
//        Intake(closeToL2!!),
//        PostIntakeApproachShoot(L2Shoot!!),
//        ShootClose,
//
//        GateIntakeClose,
//        PostIntakeApproachShoot(gateToShoot!!),
//        ShootClose,
//
//        Intake(closeToL1!!),
//        PostIntakeApproachShoot(L1Shoot!!),
//        ShootClose,
//
//        Intake(closeToL3!!),
//        PostIntakeApproachShoot(L3Shoot!!),
//        ShootClose,
//
//        Park
//    )
    private val autonomousRoutine: Command
        get() = SequentialGroup(
            // push partner
            maxPower,
            FollowPath(pushPath!!),
//            Delay(delayStartup),
            EnableSOTM,
            FollowPath(pushToClose!!),
            DisableSOTM,

            // preload
            Delay(delayPreShoot),
            ShootCommandClose,
            Delay(delayCloseShoot),

            // L2 cycle
            TravelCommand,
            FollowPath(closeToL2!!),  // intake L2
            Delay(delayAfterIntake),
            ParallelGroup(  // shoot l2
                SequentialGroup(
                    IntakeAfterCommand,
                    Delay(delayInIntake),
                    TravelCommand,
                ),
                SequentialGroup(
                    EnableSOTM,
                    FollowPath(L2Shoot!!),
                    DisableSOTM
                )
            ),
            Delay(delayPreShoot),
            ShootCommandClose,
            Delay(delayCloseShoot),
            TravelCommand,

            // gate cycle
            FollowPath(closeToGate!!),
            Delay(delayAtGate),
//            IntakeCommand,
            FollowPath(gateToAfter!!),
            Delay(delayAfterGate),
            IntakeAfterCommand,
            ParallelGroup( // travel back
                SequentialGroup(
                    Delay(delayInIntake),
                    TravelCommand,
                ),
                SequentialGroup(
                    EnableSOTM,
                    FollowPath(gateToShoot!!),
                    DisableSOTM
                )
            ),
            Delay(delayPreShoot),  // shoot
            ShootCommandClose,
            Delay(delayCloseShoot),

            // L1 cycle
            TravelCommand,
            FollowPath(closeToL1!!),
            Delay(delayAfterIntake),
            IntakeAfterCommand,
            ParallelGroup(  // shoot l1
                SequentialGroup(
                    Delay(delayInIntake),
                    TravelCommand,
                ),
                SequentialGroup(
                    EnableSOTM,
                    FollowPath(L1Shoot!!),
                    DisableSOTM
                )
            ),
            Delay(delayPreShoot),
            ShootCommandClose,
            Delay(delayCloseShoot),

            // L3 cycle
            TravelCommand,
            FollowPath(closeToL3!!),
            Delay(delayAfterIntake),
            IntakeAfterCommand,
            ParallelGroup(  // shoot l3
                SequentialGroup(
                    Delay(delayInIntake),
                    TravelCommand,
                ),
                SequentialGroup(
                    EnableSOTM,
                    FollowPath(L3Shoot!!),
                    DisableSOTM
                )
            ),
            Delay(delayPreShoot),
            ShootCommandClose,
            Delay(delayCloseShoot),

            // HPZ Cycle
//            TravelCommand,
//            FollowPath(closeToHPZ!!),
//            Delay(delayAfterIntake),
//            IntakeAfterCommand,
//            ParallelGroup(  // shoot hpz
//                SequentialGroup(
//                    Delay(delayInIntake),
//                    TravelCommand,
//                ),
//                FollowPath(HPZShoot!!)
//            ),
//            Delay(delayPreShoot),
//            ShootCommandClose,
//            Delay(delayCloseShoot),

            // Park
            TravelCommand,
            InstantCommand { stopShooterAutoAim = true },
            ShooterSubsystem.On(9999.0),
            FollowPath(parkPath!!),
        )

    override fun onUpdate() {
//        if (pathStarted && opmodeTimer!!.elapsedTime >= 29.75) {
//            PedroComponent.follower.setMaxPower(0.0)
//            PedroComponent.follower.breakFollowing()
//            IntakeMotorSubsystem.off()
//            MagMotorSubsystem.off()
//            MagblockServoSubsystem.block()
//        }

        val dx = goalX - PedroComponent.follower.pose.x
        val dy = goalY - PedroComponent.follower.pose.y
        val dxy = hypot(dx, dy)
        val dxp = dx - (1.0*PedroComponent.follower.velocity.xComponent
                + 0.05 * PedroComponent.follower.acceleration.xComponent) * (
                if (PedroComponent.follower.pose.y < BORD_Y) AutoAimConstants.distanceToTimeFar(dxy)
                else AutoAimConstants.distanceToTimeClose(dxy)
        )
        val dyp = dy - (1.0*PedroComponent.follower.velocity.yComponent
                + 0.05 * PedroComponent.follower.acceleration.yComponent) * (
                if (PedroComponent.follower.pose.y < BORD_Y) AutoAimConstants.distanceToTimeFar(dxy)
                else AutoAimConstants.distanceToTimeClose(dxy)
        )
//        val dxp = dx;
//        val dyp = dy;
        val dxyp = hypot(dxp, dyp)
        if (!stopShooterAutoAim) {
            ShooterSubsystem.AutoAim(
                dxyp * sotmFactor + dxy * (1.0 - sotmFactor),
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
        TurretPhiSubsystem.AutoAim(
            dxp * sotmFactor + dx * (1 - sotmFactor),
            dyp * sotmFactor + dy * (1 - sotmFactor),
            PedroComponent.follower.pose.heading.rad
        )();
        TurretThetaSubsystem.AutoAim(
            dxyp * sotmFactor + dxy * (1.0 - sotmFactor),
            { dist ->
                (if (PedroComponent.follower.pose.y < BORD_Y)
                    distAndVeloToThetaFar(dist, ShooterSubsystem.velocity)
                else
                    distAndVeloToThetaClose(dist, ShooterSubsystem.velocity)) + 1.5.deg
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
//        follower = Constants.createFollower(hardwareMap)

        PedroComponent.follower.setStartingPose(
            AutonPositions.Blue(AutonPositions.startPoseFarPush)
        )

        TurretPhiSubsystem.zero();
    }

    /** This method is called continuously after Init while waiting for "play".  */
    override fun onWaitForStart() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system  */
    override fun onStartButtonPressed() {
        autonomousRoutine()

        opmodeTimer!!.resetTimer()
        actionTimer!!.resetTimer()

        pathStarted = true;
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