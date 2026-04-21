package org.firstinspires.ftc.teamcode.Auton.MainAutons
import org.firstinspires.ftc.teamcode.Auton.AutonPositions

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
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.FollowPath
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.Auton.MainAutons.AutonBlueCloseArtifact24.Companion.DelayAtLever
import org.firstinspires.ftc.teamcode.Auton.MainAutons.AutonBlueCloseArtifact24.Companion.DelayFromRampIntake
import org.firstinspires.ftc.teamcode.Auton.MainAutons.AutonBlueCloseArtifact24.Companion.canShoot

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
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.distAndVeloToNewThetaClose
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.distAndVeloToNewThetaFar
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.distAndVeloToThetaClose
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.distAndVeloToThetaFar
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.distanceToTimeClose
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.distanceToTimeFar
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.distanceToVelocityClose
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.distanceToVelocityFar
import org.firstinspires.ftc.teamcode.utils.Lefile
import java.io.File
import kotlin.math.hypot
import kotlin.math.max
import kotlin.math.min

@Autonomous(name = "[COOP-21-B] Auton Blue Close CoOp", group = "Auton")
@Configurable
class AutonBlueCloseCoOp: NextFTCOpMode() {
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
        val delayStartShoot: Double = 0.4
        val DelayBeforeShoot: Double = 0.0
        val delayAfterEachShoot: Double = 0.27 //currently at a really high #
        val DelayForPartnerBot: Double = 0.0
        val DelayInIntake: Double = 0.45
        val delayAtGate: Double = 0.67

        val goalX = 2.0
        val goalY = 144.0 - 4.0

        val intakeSpeed = 0.8
//        var directionGoalX = 4.0;
//        var directionGoalY = 144.0-4.0;
    }

    /////////////////
    ////Positions////
    /////////////////
    //Constant positions
//    private val shootPoseCloseFromIntake1 = Pose(57.0, 76.6, Math.toRadians(128.0)) // Close Shoot Pose of our robot.


    /////////////
    ////Paths////
    /////////////
    // The different paths the robot will take in during Auton
    private var robotShootPreload: PathChain? = null

    private var robotIntake1: PathChain? = null
    private var robotGoToShoot1: PathChain? = null //since preload also intakes, there is no intake1

    private var robotIntake2: PathChain? = null
    private var robotGoToShoot2: PathChain? = null

    private var shootDirectToGate: PathChain? = null

    private var gateToCommon: PathChain? = null
    private var robotFirstLeverOpen: PathChain? = null //very important

    private var robotFinalCommonIntake: PathChain? = null
    private var robotFinalCommonGoShoot: PathChain? = null

    private var robotCommonIntake: PathChain? = null
    private var robotCommonGoShoot: PathChain? = null

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
                    AutonPositions.Blue(AutonPositions.start24ShootPos),
                )
            )
//tc//            .setTimeoutConstraint(delayPreShoot / 2.0)
//            .setTangentHeadingInterpolation()
            .setConstantHeadingInterpolation(AutonPositions.Blue(AutonPositions.start24ShootPos).heading)
            .build()

        robotIntake1 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.start24ShootPos),
                    AutonPositions.Blue(AutonPositions.intake1Pos24)
                )
            )
//tc//            .setTimeoutConstraint(delayAfterIntake / 2.0)
//            .setTangentHeadingInterpolation()
            .addParametricCallback(0.90, IntakeCommand) //WHERE INTAKE COMMAND WILL NOW GO IG
            .setLinearHeadingInterpolation(
                AutonPositions.Blue(AutonPositions.start24ShootPos).heading,
                AutonPositions.Blue(AutonPositions.intake1Pos24).heading,
                0.9
            )
            .build()

        robotFirstLeverOpen = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    AutonPositions.Blue(AutonPositions.intake1Pos24),
                    AutonPositions.Blue(AutonPositions.coOpFirstGateOpenControlPos),
                    AutonPositions.Blue(AutonPositions.coOpFirstStartGateOpen),
                )
            )
            .setLinearHeadingInterpolation(
                AutonPositions.Blue(AutonPositions.intake1Pos24).heading,
                AutonPositions.Blue(AutonPositions.coOpFirstStartGateOpen).heading,
                0.5
            )
            .setConstantHeadingInterpolation(
                AutonPositions.Blue(AutonPositions.coOpFirstStartGateOpen).heading
            )
            .build()

        //1st Go Shoot
        robotGoToShoot1 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.coOpFirstStartGateOpen),
                    AutonPositions.Blue(AutonPositions.shootPoseClose)
                )
            )
            .addParametricCallback(0.0, IntakeAfterCommand)
            .addParametricCallback(0.5, TravelCommand)
//tc//            .setTimeoutConstraint(delayPreShoot / 2.0)
            .setHeadingInterpolation(
                HeadingInterpolator.tangent.reverse()
            )
            .build()

        //2nd Intake
        robotIntake2 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    AutonPositions.Blue(AutonPositions.shootPoseClose),
                    AutonPositions.Blue(AutonPositions.coOpStartGateOpenControlPos),
                    AutonPositions.Blue(AutonPositions.coOpStartGateOpen)
                )
            )
            .addParametricCallback(0.6, IntakeCommand) //WHERE INTAKE COMMAND WILL NOW GO IG
            .addParametricCallback(0.95, IntakeAfterCommand)
            .setLinearHeadingInterpolation(
                AutonPositions.Blue(Math.toRadians(210.0)),
                AutonPositions.Blue(Math.toRadians(180.0)),
                0.9
            )
//tc//            .setTimeoutConstraint(delayAfterIntake / 2.0)
//            .setConstantHeadingInterpolation(
//                AutonPositions.Blue(AutonPositions.coOpStartGateOpen).heading
//            )
            .build()

        shootDirectToGate = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.shootPoseClose),
                    AutonPositions.Blue(AutonPositions.coOpStartGateOpen)
                )
            )
//            .addParametricCallback(0.80, IntakeCommand) //WHERE INTAKE COMMAND WILL NOW GO IG
            .setConstantHeadingInterpolation(
                AutonPositions.Blue(AutonPositions.coOpStartGateOpen).heading
            )
            .build()

        //2nd Go Shoot
        robotGoToShoot2 = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.coOpStartGateOpen),
                    AutonPositions.Blue(AutonPositions.shootPoseClose)
                )
            )
//tc//            .setTimeoutConstraint(delayPreShoot / 2.0)
            .setConstantHeadingInterpolation(
                AutonPositions.Blue(AutonPositions.shootPoseClose).heading
            )
            .build()

        gateToCommon = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    AutonPositions.Blue(AutonPositions.coOpStartGateOpen),
                    AutonPositions.Blue(AutonPositions.coOpGateToCommonControlPos),
                    AutonPositions.Blue(AutonPositions.CoOpCommonIntake)
                )
            )
            .addParametricCallback(0.90, IntakeCommand)
//tc//            .setTimeoutConstraint(delayAfterIntake / 2.0)
            .setConstantHeadingInterpolation(AutonPositions.Blue(AutonPositions.coOpStartGateOpen).heading)
            .build()

        //3rd Intake
//        robotCommonIntake = PedroComponent.follower.pathBuilder()
//            .addPath(
//                BezierCurve(
//                    AutonPositions.Blue(AutonPositions.shootPoseClose),
//                    AutonPositions.Blue(AutonPositions.CoOpCommonIntakeControlPos1),
//                    AutonPositions.Blue(AutonPositions.CoOpCommonIntakeControlPos2),
//                    AutonPositions.Blue(AutonPositions.CoOpCommonIntake)
//                )
//            )
//            .addParametricCallback(0.90, IntakeCommand) //WHERE INTAKE COMMAND WILL NOW GO IG
//            .setConstantHeadingInterpolation(AutonPositions.Blue(AutonPositions.CoOpCommonIntake).heading)
//            .build()

        //3rd Go Shoot
        robotCommonGoShoot = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.CoOpCommonIntake),
                    AutonPositions.Blue(AutonPositions.shootPoseClose)
                )
            )
//tc//            .setTimeoutConstraint(delayPreShoot / 2.0)
            .setConstantHeadingInterpolation(
                AutonPositions.Blue(AutonPositions.shootPoseClose.heading)
            )
            .build()

        robotFinalCommonIntake = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    AutonPositions.Blue(AutonPositions.shootPoseClose),
                    AutonPositions.Blue(AutonPositions.CoOpFinalCommonControl),
                    AutonPositions.Blue(AutonPositions.CoOpFinalCommonIntake),
                )
            )
//tc//            .setTimeoutConstraint(delayAfterIntake / 2.0)
            .addParametricCallback(0.9, IntakeCommand)
            .setTangentHeadingInterpolation()
            .build()

        robotFinalCommonGoShoot = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierCurve(
                    AutonPositions.Blue(AutonPositions.CoOpFinalCommonIntake),
                    AutonPositions.Blue(AutonPositions.CoOpFinalCommonControl),
                    AutonPositions.Blue(AutonPositions.shootPoseClose)
                )
            )
//tc//            .setTimeoutConstraint(delayPreShoot / 2.0)
            .setConstantHeadingInterpolation(
                AutonPositions.Blue(AutonPositions.shootPoseClose.heading)
            )
            .build()

        //Go Park
        robotPark = PedroComponent.follower.pathBuilder()
            .addPath(
                BezierLine(
                    AutonPositions.Blue(AutonPositions.shootPoseClose),
                    AutonPositions.Blue(AutonPositions.autonParkPose)
                )
            )
//tc//            .setTimeoutConstraint(0.0)
            .setConstantHeadingInterpolation(
                AutonPositions.Blue(
                    AutonPositions.autonParkPose.heading
                )
            )
            .build()
    }

    ///////////////////////////
    ////Main Auton Commands////
    ///////////////////////////
    val intakePower: Command = InstantCommand {PedroComponent.follower.setMaxPower(1.0)}
    val maxPower: Command = InstantCommand {PedroComponent.follower.setMaxPower(1.0)}

//    val SetCanShootFalse: Command = InstantCommand {canShoot = false}
//    val SetCanShootTrue: Command = InstantCommand {canShoot = true}

    val stopFollower: Command = InstantCommand {PedroComponent.follower.breakFollowing()}
    val pauseFollower: Command = InstantCommand {PedroComponent.follower.pausePathFollowing()}

    val IntakeCommand: Command
        get() = ParallelGroup(
//            SetCanShootTrue,
            intakePower,
            IntakeMotorSubsystem.intake,
            MagMotorSubsystem.intake,
//            MagServoSubsystem.run,
            MagblockServoSubsystem.block

        )
    val IntakeAfterCommand: Command
        get() = ParallelGroup(
//            SetCanShootTrue,
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
            MagblockServoSubsystem.unblock,

            )
    val ShootCommand: Command
        get() = ParallelGroup(
//            SetCanShootFalse,
            maxPower,
            MagblockServoSubsystem.unblock,
            MagMotorSubsystem.On(1.0),
            IntakeMotorSubsystem.intake,
//            MagServoSubsystem.run
        )

    fun robotShoot(): Command {
        return SequentialGroup(
            stopFollower,
            pauseFollower,
            Delay(DelayBeforeShoot),
            ShootCommand,
            Delay(delayAfterEachShoot),
            TravelCommand,
        )
    }

    fun robotIntake(followedPath: PathChain?): Command {
        return SequentialGroup(
            TravelCommand,
            FollowPath(followedPath!!)
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

    /////////////////////////
    ////Autonomous Runner////
    /////////////////////////
    private val autonomousRoutine: Command
        get() = SequentialGroup( //Main Group
            FollowPath(robotShootPreload!!),

            ParallelGroup(
                Delay(1.0),
                ShootCommand,
            ),

            robotIntake(robotIntake1),
            Delay(0.5),

            robotGoShoot(robotFirstLeverOpen),
            Delay(delayAtGate),

            robotGoShoot(robotGoToShoot1),

//            Delay(delayAtGate),
//            robotIntake(robotIntake2),
            robotGoShoot(robotGoToShoot2),

            // NEW

            // ADD BELOW IF TIME
//            SequentialGroup(
//                Delay(DelayBeforeShoot),
//                ShootCommand,
//                Delay(delayAfterEachShoot),
//                TravelCommand,
//                FollowPath(shootDirectToGate!!),
//                TravelCommand
//            ),
//
//            SequentialGroup(
//                FollowPath(gateToCommon!!),
//            ),
//
//            ParallelGroup( //Robot goes back to FAR Shoot Position
//                SequentialGroup(
//                    IntakeAfterCommand,
//                    Delay(DelayInIntake),
//                    TravelCommand
//                ),
//                FollowPath(robotCommonGoShoot!!)
//            ),

            // END NEW

//            SequentialGroup( //Shoots SECOND Intake, goes to intake and then OPEN lever
//                Delay(DelayBeforeShoot),
//                ShootCommand,
//                Delay(delayAfterEachShoot),
//                TravelCommand,
//                FollowPath(robotIntake2!!), //robot goes to intake
//            ),
//
//            ParallelGroup( //Robot goes back to FAR Shoot Position
//                SequentialGroup(
//                    IntakeAfterCommand,
//                    Delay(DelayInIntake),
//                ),
//                FollowPath(robotGoToShoot2!!)
//            ),

            //////////////////////////////////
            ////////REPEATABLE SECTION////////
            //////////////////////////////////
            SequentialGroup( //Shoots THIRD Intake, goes to OPEN lever and then intake
                FollowPath(shootDirectToGate!!),
                TravelCommand
            ),

            FollowPath(gateToCommon!!),
            robotGoShoot(robotCommonGoShoot),

            SequentialGroup( //Shoots THIRD Intake, goes to OPEN lever and then intake
                Delay(DelayForPartnerBot), //Temporary Pos for this delay until figure out how needed or changed delay has to be
                FollowPath(robotFinalCommonIntake!!), //robot goes to intake
            ),

            robotGoShoot(robotFinalCommonGoShoot),

            //////////////////////////////////
            ////////REPEATABLE SECTION////////
            //////////////////////////////////
            SequentialGroup( //Shoots THIRD Intake, goes to OPEN lever and then intake
                FollowPath(shootDirectToGate!!),
                TravelCommand
            ),

            FollowPath(gateToCommon!!),
            robotGoShoot(robotCommonGoShoot),

            SequentialGroup( //Shoots THIRD Intake, goes to OPEN lever and then intake
                Delay(DelayForPartnerBot), //Temporary Pos for this delay until figure out how needed or changed delay has to be
                FollowPath(robotFinalCommonIntake!!), //robot goes to intake
            ),

            robotGoShoot(robotFinalCommonGoShoot),

            //////////////////////////////////
            ////////REPEATABLE SECTION////////
            //////////////////////////////////


            SequentialGroup( //Shoots SIXTH Intake, goes to PARK
                InstantCommand { stopShooterAutoAim = true },
                ShooterSubsystem.On(9999.0),
                FollowPath(robotPark!!), //robot goes to intake
            ),
        )

    private var stopShooterAutoAim = false;
    var vxOld = listOf(0.0, 0.0, 0.0);
    var vyOld = listOf(0.0, 0.0, 0.0);
    var lastPose: Pose = Pose(0.0, 0.0, 0.0);
    var lastTime = 0.0;
    var lastInTriangle = 0.0;
    override fun onUpdate() {
        val dx = AutonBlueCloseArtifact24.Companion.goalX - PedroComponent.follower.pose.x
        val dy = AutonBlueCloseArtifact24.Companion.goalY - PedroComponent.follower.pose.y
        val dxy = hypot(dx, dy)
        if (vxOld.isEmpty() || vyOld.isEmpty()) {
            vxOld = listOf(0.0, 0.0, 0.0);
            vyOld = listOf(0.0, 0.0, 0.0);
        } else {
            vxOld = vxOld.slice(IntRange(1, vxOld.lastIndex)) + listOf((PedroComponent.follower.pose.x - lastPose.pose.x) / (runtime - lastTime));
            vyOld = vyOld.slice(IntRange(1, vyOld.lastIndex)) + listOf((PedroComponent.follower.pose.y - lastPose.pose.y) / (runtime - lastTime));
        }
        val vx = vxOld.average();
        val vy = vyOld.average();
        val dxp = dx - 1.0 * vx * (if (PedroComponent.follower.pose.y < BORD_Y) distanceToTimeFar(dxy) else distanceToTimeClose(dxy))
        val dyp = dy - 1.0 * vy * (if (PedroComponent.follower.pose.y < BORD_Y) distanceToTimeFar(dxy) else distanceToTimeClose(dxy))
        val dxyp = hypot(dxp, dyp)
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
        TurretThetaSubsystem.SetThetaPos(
            (
                    if (PedroComponent.follower.pose.y < BORD_Y)
                        distAndVeloToNewThetaFar(dxyp, ShooterSubsystem.velocity)
                    else
                        distAndVeloToNewThetaClose(dxyp, ShooterSubsystem.velocity)
                    )
        )()
        TurretPhiSubsystem.AutoAim(
            dxp,
            dyp,
            PedroComponent.follower.heading.rad
        )()
        lastPose = PedroComponent.follower.pose;
        lastTime = runtime

        val inTriangle = inTriangle(PedroComponent.follower.pose.x, PedroComponent.follower.pose.y, 6.0);
        if (inTriangle >= 1 && canShoot) {
            canShoot = false
            robotShoot()()
        }

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

    fun inTriangle(x1: Double, y1: Double, margin: Double): Int {
        /**0 = none, 1 = top, 2 = bottom, -1 = error */
        if (x1 > 144 || x1 < 0 || y1 > 144 || y1 < 0) {
            return -1
        }

        // T triangle: vertices (72,64), (-8,144), (152,144)
        val inTop = (y1 >= -x1 + 144 - margin) && (y1 >= x1 - margin)

        // B triangle:  (40,0), (72,32), (104,0)
        val inBottom = (y1 <= x1 - (48 - margin)) && (y1 <= -x1 + 96 + margin)

        if (inTop) return 1
        if (inBottom) return 2
        return 0
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

        TurretThetaSubsystem.SetThetaPos(0.63 + Math.random() * 0.01)()

        PedroComponent.follower.setStartingPose(AutonPositions.Blue(AutonPositions.startPoseClose))

        PedroComponent.follower.update()

        val dx = Companion.goalX - PedroComponent.follower.pose.x
        val dy = Companion.goalY - PedroComponent.follower.pose.y
        TurretPhiSubsystem.AutoAim(
            dx + Math.random() * 0.2,
            dy + Math.random() * 0.2,
            PedroComponent.follower.heading.rad
        )()
    }

    /** This method is called continuously after Init while waiting for "play".  */
    override fun onWaitForStart() {}

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