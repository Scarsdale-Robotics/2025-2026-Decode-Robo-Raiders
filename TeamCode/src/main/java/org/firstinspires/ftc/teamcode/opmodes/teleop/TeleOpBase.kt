package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.driving.FieldCentric
import dev.nextftc.hardware.driving.MecanumDriverControlled
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.opmodes.teleop.BasicTeleOp.Companion.isBlue
import org.firstinspires.ftc.teamcode.opmodes.teleop.BasicTeleOp.Companion.shootAngleDegrees
import org.firstinspires.ftc.teamcode.opmodes.teleop.BasicTeleOp.Companion.speed1
import org.firstinspires.ftc.teamcode.subsystems.LowerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometrySubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.LowerMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagblockServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem
import java.io.File
import java.util.function.Supplier
import kotlin.math.PI
import kotlin.math.hypot

@Configurable
class TeleOpBase(
    private val isBlue: Boolean,
    private val goalX: Double,
    private val goalY: Double,
//    private val distanceToFlightTimeSecs: (Double) -> Double,
//    private val invertDriveControls: Boolean,
    private val distanceToVelocity: (Double) -> Double,
    private val distanceToTheta: (Double) -> Angle
): NextFTCOpMode() {

    private val lfw = MotorEx("lfw").reversed();
    private val lbw = MotorEx("lbw").reversed();
    private val rfw = MotorEx("rfw");
    private val rbw = MotorEx("rbw");

    private var odom: OdometrySubsystem? = null;

    private var ofsX = 0.0;
    private var ofsY = 0.0;
    private var ofsH = 0.0;

    val x: Double
        get() {
            if (odom != null) {
                return odom!!.rOx1 + ofsX;
            }
            return 0.0;
        }
    val y: Double
        get() {
            if (odom != null) {
                return odom!!.rOy1 + ofsY;
            }
            return 0.0;
        }
    val h: Angle
        get() {
            if (odom != null) {
                return odom!!.rOh.rad + ofsH.rad;
            }
            return 0.0.rad;
        }

    override fun onInit() {
        addComponents(
            SubsystemComponent(
                LowerSubsystem,
                OuttakeSubsystem,
                MagServoSubsystem,
                MagblockServoSubsystem,
                TurretThetaSubsystem
            ),
            BindingsComponent,
            BulkReadComponent
        )

        ShooterSubsystem.off()
        LowerMotorSubsystem.off()
        MagServoSubsystem.stop()

        odom = OdometrySubsystem(72.0, 72.0, PI / 2, hardwareMap)
    }

    private var autoAimEnabled = true;
    var speedFactor = 1.0;
    override fun onStartButtonPressed() {
//        val file = File("RobotAutonEndPos.txt")
//        val content = file.readText().split("\n")
//        val startX = content[0].toDouble()
//        val startY = content[1].toDouble()
//        val startH = content[2].toDouble()
//
//        odom!!.setPinpoint(startX, startY, startH)
        odom!!.setPinpoint(72.0, 72.0, PI / 2)

        MagblockServoSubsystem.unblock()
        MagblockServoSubsystem.block()

        val mecanum = MecanumDriverControlled(
            lfw,
            rfw,
            lbw,
            rbw,
            -Gamepads.gamepad1.leftStickY.map { it*speedFactor },
            Gamepads.gamepad1.leftStickX.map { it*speedFactor },
            Gamepads.gamepad1.rightStickX.map { it*speedFactor },
            FieldCentric({
                if (isBlue) (h.inRad - PI).rad else h
            })
        )
        mecanum();

        Gamepads.gamepad1.rightBumper whenBecomesTrue { speedFactor = 0.5; }
        Gamepads.gamepad1.rightBumper whenBecomesFalse { speedFactor = 1.0; }

        val lowerMotorDrive = LowerMotorSubsystem.DriverCommandDefaultOn(
            Gamepads.gamepad1.leftTrigger
        );
        lowerMotorDrive();

        val magDrive = MagServoSubsystem.DriverCommandDefaultOn(
            Gamepads.gamepad1.leftTrigger.greaterThan(0.0)
        )
        magDrive();

        Gamepads.gamepad1.circle
            .whenTrue(MagblockServoSubsystem.unblock)
            .whenBecomesFalse(MagblockServoSubsystem.block)

        Gamepads.gamepad2.leftBumper and Gamepads.gamepad2.rightBumper whenBecomesTrue {
            autoAimEnabled = !autoAimEnabled;
            gamepad1.rumble(450);
            gamepad2.rumble(450);
        }

        val dx = Supplier { goalX - x }
        val dy = Supplier { goalY - y }
        val dxy = Supplier { hypot(dx.get(), dy.get()) }

        if (autoAimEnabled) {
            ShooterSubsystem.AutoAim(
                dxy,
                distanceToVelocity
            )
            TurretPhiSubsystem.AutoAim(
                dx, dy, { h }
            )
            TurretThetaSubsystem.AutoAim(
                dxy,
                distanceToTheta
            )
        } else {
            //ShooterSubsystem.Manual(

            //)
        }
    }

    override fun onUpdate() {
        odom!!.updateOdom()

        telemetry.addData("x (inch)", odom!!.rOx1);
        telemetry.addData("y (inch)", odom!!.rOy1);
        telemetry.addData("h (radians)", odom!!.rOh);
        telemetry.addData(
            "distanceToGoal",
            hypot((3 - odom!!.rOx1), (141 - odom!!.rOy1))
        );
        telemetry.addData("ShooterSpeed", speed1);
        telemetry.addData("Angle", shootAngleDegrees.deg);
        telemetry.update()
        PanelsTelemetry.telemetry.update()
    }
}