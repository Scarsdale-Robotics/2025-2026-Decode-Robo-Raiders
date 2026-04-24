package org.firstinspires.ftc.teamcode.Auton.MainAutons

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierCurve
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.HeadingInterpolator
import com.pedropathing.paths.PathBuilder
import com.pedropathing.paths.PathChain
import com.pedropathing.util.Timer
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
import org.firstinspires.ftc.teamcode.Auton.AutonPositions
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.LowerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagblockServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.BORD_Y
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.distAndVeloToNewThetaClose
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.distAndVeloToNewThetaFar
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.distanceToTimeClose
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.distanceToTimeFar
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.distanceToVelocityClose
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants.distanceToVelocityFar
import org.firstinspires.ftc.teamcode.utils.Lefile
import java.io.File
import kotlin.math.hypot

object AutonUtil {
    var canShoot = false;

    val intakePower: Command = InstantCommand {PedroComponent.follower.setMaxPower(0.5)}
    val maxPower: Command = InstantCommand {PedroComponent.follower.setMaxPower(1.0)}

    val SetCanShootFalse: Command = InstantCommand {canShoot = false}
    val SetCanShootTrue: Command = InstantCommand {canShoot = true}

    val stopFollower: Command = InstantCommand {PedroComponent.follower.breakFollowing()}
    val pauseFollower: Command = InstantCommand {PedroComponent.follower.pausePathFollowing()}
    val unPauseFollower: Command = InstantCommand {PedroComponent.follower.resumePathFollowing()}

    val IntakeCommand = ParallelGroup(
        SetCanShootTrue,
//        intakePower,
        maxPower,  // todo: test
        MagblockServoSubsystem.block,
        SequentialGroup(
            Delay(0.3),
            IntakeMotorSubsystem.intake,
            MagMotorSubsystem.intake,
        )

    )
    val IntakeAfterCommand = ParallelGroup(
        SetCanShootTrue,
        maxPower,
        IntakeMotorSubsystem.intake,
        MagMotorSubsystem.intake,
        MagblockServoSubsystem.block
    )
    val IntakeAfterCommandFinal = ParallelGroup(
        maxPower,
        IntakeMotorSubsystem.intake,
        MagMotorSubsystem.intake,
        MagblockServoSubsystem.block
    )
    val TravelCommand = ParallelGroup(
        maxPower,
        IntakeMotorSubsystem.off,
        MagMotorSubsystem.off,
        MagblockServoSubsystem.unblock,
    )
    val ShootCommand = ParallelGroup(
        maxPower,
        MagblockServoSubsystem.unblock,
        MagMotorSubsystem.On(1.0),
        IntakeMotorSubsystem.intake,
    )

    val delayStartShoot: Double = 1.2
    val delayBeforeShoot: Double = 0.19
    val delayAfterEachShoot: Double = 0.38 //currently at a really high #
    val delayFromRampIntake: Double = 0.66000000676767676767676767
    val delayInIntake: Double = 0.5
    val delayAtLever: Double = 0.0

    val goalX = 5.0
    val goalY = 144.0 - 5.0


    fun robotShoot(): Command {
        return SequentialGroup(
//            pauseFollower,
//            stopFollower,
            Delay(delayBeforeShoot),
            ShootCommand,
            Delay(delayAfterEachShoot),
//            unPauseFollower,
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
//            Delay(delayAtLever),
            IntakeCommand,
            FollowPath(goBackPath!!),
            Delay(delayFromRampIntake),
        )
    }
    fun robotGateIntakeOneShot(followedPath: PathChain): Command {
        return SequentialGroup(
            TravelCommand,
            FollowPath(followedPath),
            Delay(delayFromRampIntake),
        )
    }
    fun finalGoShoot(followedPath: PathChain?): Command {
        return ParallelGroup(
            SequentialGroup(
                IntakeAfterCommandFinal,
                Delay(delayInIntake),
                TravelCommand,
            ),
            FollowPath(followedPath!!, true)
        )
    }
    fun robotGoShoot(followedPath: PathChain?): Command {
        return ParallelGroup(
            SequentialGroup(
                IntakeAfterCommand,
                Delay(delayInIntake),
                TravelCommand,
            ),
            FollowPath(followedPath!!, true)
        )
    }

        fun parkRobot(followedPath: PathChain?): Command {
        return SequentialGroup(
//            InstantCommand { stopShooterAutoAim = true },
            ShooterSubsystem.On(9999.0),
            FollowPath(followedPath!!, true)
        )
    }
}

@Configurable
open class AutonBase(
    private val isBlue: Boolean,
    private val goalX: Double,
    private val goalY: Double,
    private val startPose: Pose,
    private val autonomousRoutine: (Boolean, Follower) -> Command,
) : NextFTCOpMode() {
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

    var vxOld = listOf(0.0, 0.0, 0.0);
    var vyOld = listOf(0.0, 0.0, 0.0);
    var axOld = listOf(0.0, 0.0, 0.0);
    var ayOld = listOf(0.0, 0.0, 0.0);
    var lastPose: Pose = Pose(0.0, 0.0, 0.0);
    var lastVX = 0.0;
    var lastVY = 0.0;
    var lastTime = 0.0;
    var lastInTriangle = 0.0;
    override fun onUpdate() {
        val dx = goalX - PedroComponent.follower.pose.x
        val dy = goalY - PedroComponent.follower.pose.y
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
        if (axOld.isEmpty() || ayOld.isEmpty()) {
            axOld = listOf(0.0, 0.0, 0.0);
            ayOld = listOf(0.0, 0.0, 0.0);
        } else {
            axOld = axOld.slice(IntRange(1, axOld.lastIndex)) + listOf((vx - lastVX) / (runtime - lastTime));
            ayOld = ayOld.slice(IntRange(1, ayOld.lastIndex)) + listOf((vy - lastVY) / (runtime - lastTime));
        }
        val ax = axOld.average();
        val ay = ayOld.average();
        val timeFactor = (if (PedroComponent.follower.pose.y < BORD_Y) distanceToTimeFar(dxy) else distanceToTimeClose(dxy))
        val dxp = { accelFactor: Double -> dx - 1.0 * vx * timeFactor - accelFactor * ax * timeFactor * timeFactor }
        val dyp = { accelFactor: Double -> dy - 1.0 * vy * timeFactor - accelFactor * ay * timeFactor * timeFactor }
        val dxyp = { accelFactor: Double -> hypot(dxp(accelFactor), dyp(accelFactor)) }
        ShooterSubsystem.AutoAim(
            dxyp(0.0),
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
                        distAndVeloToNewThetaFar(dxyp(0.0), ShooterSubsystem.velocity)
                    else
                        distAndVeloToNewThetaClose(dxyp(0.0), ShooterSubsystem.velocity)
                    )
        )()
        TurretPhiSubsystem.AutoAim(
            dxp(0.0),
            dyp(0.0),
            PedroComponent.follower.heading.rad
        )()
        lastPose = PedroComponent.follower.pose;
        lastVX = vx;
        lastVY = vy;
        lastTime = runtime

//        val inTriangle = inTriangle(PedroComponent.follower.pose.x, PedroComponent.follower.pose.y, 6.0);
//        if (inTriangle >= 1 && AutonUtil.canShoot) {
//            AutonUtil.canShoot = false
//            AutonUtil.robotShoot()()
//        }

        telemetry.addData("canShoot", AutonUtil.canShoot)
        telemetry.addData("InTriangle", inTriangle(PedroComponent.follower.pose.x, PedroComponent.follower.pose.y, 8.0))
        telemetry.addData("x", PedroComponent.follower.pose.x)
        telemetry.addData("y", PedroComponent.follower.pose.y)
        telemetry.addData("heading", PedroComponent.follower.pose.heading)
        telemetry.addData("shooter necessary", distanceToVelocityFar(dxyp(0.0)))
        telemetry.update()

        PanelsTelemetry.telemetry.update()
    }

    /** This method is called continuously after Init while waiting for "play". **/
    override fun onInit() {
        AutonUtil.canShoot = false

        ShooterSubsystem.off()
        IntakeMotorSubsystem.off()
        MagMotorSubsystem.off()
        MagblockServoSubsystem.block()
        TurretThetaSubsystem.SetThetaPos(0.63 + Math.random() * 0.01)()

        PedroComponent.follower.setStartingPose(startPose)
        PedroComponent.follower.update()

        val dx = goalX - PedroComponent.follower.pose.x
        val dy = goalY - PedroComponent.follower.pose.y
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
        autonomousRoutine(isBlue, PedroComponent.follower)()
        pathStarted = true

        opmodeTimer!!.resetTimer()
        actionTimer!!.resetTimer()
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