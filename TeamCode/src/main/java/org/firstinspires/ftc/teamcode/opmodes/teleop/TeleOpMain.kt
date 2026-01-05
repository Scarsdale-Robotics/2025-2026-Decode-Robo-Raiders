package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.bylazar.telemetry.TelemetryManager
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.Path
import com.pedropathing.paths.PathChain
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.subsystems.LowerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.intake.IntakeMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.intake.IntakeServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.MagazineMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.MagblockServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.PusherServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem
import org.firstinspires.ftc.teamcode.utils.Lefile.filePath
import java.io.File
import java.util.function.Supplier
import kotlin.math.hypot


@Configurable
open class TeleOpMain(
    private val isRed: Boolean,
    private val goalX: Double,
    private val goalY: Double,
    private val distanceToFlightTimeSecs: (Double) -> Double,
    private val invertDriveControls: Boolean,
    private val distanceToVelocity: (Double) -> Double,
    private val distanceToTheta: (Double) -> Angle,
    private val boxBoxPose: Pose,
    private val hammerTimePose: Pose,
    private val closeShootPose: Pose,
    private val farShootPose: Pose
) : NextFTCOpMode() {

    companion object {
        var dtCalcIterations = 100;
    }

    private val follower: Follower

//    private var odom: OdometrySubsystem? = null;

    private val lfw = MotorEx("lfw").reversed();
    private val lbw = MotorEx("lbw").reversed();
    private val rfw = MotorEx("rfw");
    private val rbw = MotorEx("rbw");

    private var ofsX = 0.0;
    private var ofsY = 0.0;
    private var ofsH = 0.0;

    val x: Double
        get() {
            return follower.pose.x + ofsX;
        }
    val y: Double
        get() {
            return follower.pose.y + ofsY;
        }
    val h: Angle
        get() {
            return follower.pose.heading.rad + ofsH.rad;
        }
    val vx: Double
        get() {
            return follower.velocity.xComponent;
        }
    val vy: Double
        get() {
            return follower.velocity.yComponent;
        }
    val vh: Angle
        get() {
            return follower.velocity.theta.rad;
        }

    private var speedFactor = 1.0;

    private val telemetryM: TelemetryManager

    private val boxBoxPath: Supplier<PathChain>
    private val hammerTimePath: Supplier<PathChain>
    private val farShootPath: Supplier<PathChain>
    private val closeShootPath: Supplier<PathChain>

    init {
        val file = File("RobotAutonEndPos.txt")
        val content = file.readText().split("\n")
        val startX = content[0].toDouble()
        val startY = content[1].toDouble()
        val startH = content[2].toDouble()

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(Pose(startX, startY, startH))
        follower.update();

        telemetryM = PanelsTelemetry.telemetry;

        // pedro paths
        boxBoxPath = Supplier {
            follower.pathBuilder()
                .addPath(Path(BezierLine(
                    follower::getPose, boxBoxPose
                )))
                .build()
        }
        hammerTimePath = Supplier {
            follower.pathBuilder()
                .addPath(Path(BezierLine(
                    follower::getPose, hammerTimePose
                )))
                .build()
        }
        farShootPath = Supplier {
            follower.pathBuilder()
                .addPath(Path(BezierLine(
                    follower::getPose, farShootPose
                )))
                .build()
        }
        closeShootPath = Supplier {
            follower.pathBuilder()
                .addPath(Path(BezierLine(
                    follower::getPose, closeShootPose
                )))
                .build()
        }
    }

    override fun onInit() {
        addComponents(
            SubsystemComponent(
                LowerSubsystem,
                OuttakeSubsystem
            ),
            BindingsComponent,
            BulkReadComponent
        )

        ShooterSubsystem.off()
        MagazineMotorSubsystem.off()
        IntakeMotorSubsystem.off()
    }

    override fun onStartButtonPressed() {
        IntakeServoSubsystem.up()
        PusherServoSubsystem.out()
        MagblockServoSubsystem.close()
        MagazineMotorSubsystem.slow()

        // DRIVER CONTROLS
        // Drivetrain
        val driveCommand = PedroDriverControlled(
            Gamepads.gamepad1.leftStickY.map { it * speedFactor * (if (invertDriveControls) -1.0 else 1.0) },
            Gamepads.gamepad1.leftStickX.map { it * speedFactor * (if (invertDriveControls) 1.0 else -1.0) },
            Gamepads.gamepad1.rightStickX.map { it * speedFactor },
            false
        )
        Gamepads.gamepad1.rightBumper whenBecomesTrue { speedFactor = 0.5; }
        Gamepads.gamepad1.rightBumper whenBecomesFalse { speedFactor = 1.0; }

        Gamepads.gamepad1.dpadUp whenBecomesTrue { follower.followPath(hammerTimePath.get()) }
        Gamepads.gamepad1.dpadDown whenBecomesTrue { follower.followPath(boxBoxPath.get()) }
        (if (invertDriveControls) Gamepads.gamepad1.dpadRight else Gamepads.gamepad1.dpadLeft) whenBecomesTrue { follower.followPath(closeShootPath.get()) }
        (if (invertDriveControls) Gamepads.gamepad1.dpadLeft else Gamepads.gamepad1.dpadRight) whenBecomesTrue { follower.followPath(farShootPath.get()) }

        // Scoring
        // g1circle -> open shoot
        Gamepads.gamepad1.circle whenBecomesTrue SequentialGroup(
            MagblockServoSubsystem.open,
            MagazineMotorSubsystem.fast  // also enters with gradually faster speed helps counter flywheel speed loss
        ) whenBecomesFalse SequentialGroup(
            MagazineMotorSubsystem.slow,
            MagblockServoSubsystem.close
        )

        // Intake
        Gamepads.gamepad2.triangle whenBecomesTrue ParallelGroup(
            IntakeServoSubsystem.down,
            IntakeMotorSubsystem.intake
        ) whenBecomesFalse ParallelGroup(
            IntakeServoSubsystem.up,
            IntakeMotorSubsystem.slow
        )

        // AUTO AIM
        val dx = Supplier { goalX - x }
        val dy = Supplier { goalY - y }
        val dxy = Supplier { hypot(dx.get(), dy.get()) }
        val dt = Supplier {
            var dtLastIter = distanceToFlightTimeSecs(dxy.get())
            for (i in 1..dtCalcIterations) {
                val estgRprime = distanceToFlightTimeSecs(
                    hypot(
                        x + vx*dtLastIter - goalX,
                        y + vy*dtLastIter - goalY
                    )
                )
                val xIter = (goalX - x) / (vx - ((goalX - x - vx * dtLastIter) / estgRprime))
                val yIter = (goalY - y) / (vy - ((goalY - y - vy * dtLastIter) / estgRprime))
                dtLastIter = hypot(xIter, yIter)
            }
            return@Supplier dtLastIter
        }
        val xp = Supplier { x + vx * dt.get() }
        val yp = Supplier { y + vy * dt.get() }
        val hp = Supplier { (h.inRad + vh.inRad * dt.get()).rad }
        val dxyp = Supplier { hypot(xp.get(), yp.get()) }
        val dxp = Supplier { goalX - xp.get() }
        val dyp = Supplier { goalY - yp.get() }
        ShooterSubsystem.AutoAim(
            dxyp,
            distanceToVelocity
        )
        TurretPhiSubsystem.AutoAim(
            dxp, dyp, hp
        )
        TurretThetaSubsystem.AutoAim(
            dxyp,
            distanceToTheta
        )

    }

    override fun onUpdate() {
        follower.update()
        telemetryM.update()
    }

    override fun onStop() {
        val file = File(filePath)
        file.writeText(
            x.toString() + "\n" +
                    y.toString() + "\n" +
                    h.inRad.toString() + "\n"
        )
    }

}