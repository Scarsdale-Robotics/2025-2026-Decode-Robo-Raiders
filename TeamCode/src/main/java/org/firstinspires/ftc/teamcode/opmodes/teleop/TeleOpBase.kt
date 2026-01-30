package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
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
import org.firstinspires.ftc.teamcode.opmodes.teleop.BasicTeleOp.Companion.shootAngleDegrees
import org.firstinspires.ftc.teamcode.opmodes.teleop.BasicTeleOp.Companion.speed1
import org.firstinspires.ftc.teamcode.subsystems.LowerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometrySubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagblockServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem
import org.firstinspires.ftc.teamcode.utils.Lefile
import java.io.File
import java.util.function.Supplier
import kotlin.math.PI
import kotlin.math.hypot

data class ResetModeParams(val x: Double, val y: Double, val h: Angle)

@Configurable
open class TeleOpBase(
    private val isBlue: Boolean,
    private val goalX: Double,
    private val goalY: Double,
    private val resetModeParams: ResetModeParams,
//    private val distanceToFlightTimeSecs: (Double) -> Double,
//    private val invertDriveControls: Boolean,
    private val distanceToVelocity: (Double) -> Double,
    private val distanceToTheta: (Double) -> Angle,
    private val distanceToTime: (Double) -> Double
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
    val vx: Double
        get() {
            if (odom != null) {
                return odom!!.vx;
            }
            return 0.0;
        }
    val vy: Double
        get() {
            if (odom != null) {
                return odom!!.vy;
            }
            return 0.0;
        }
    val vh: Angle
        get() {
            if (odom != null) {
                return odom!!.omega.rad;
            }
            return 0.0.rad;
        }

    init {
        addComponents(
            SubsystemComponent(
                IntakeMotorSubsystem,
                MagMotorSubsystem,
                MagblockServoSubsystem,
                MagServoSubsystem,
                ShooterSubsystem,
                TurretPhiSubsystem,
                TurretThetaSubsystem
            ),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onInit() {
        ShooterSubsystem.off()
        MagMotorSubsystem.off()
        MagServoSubsystem.stop()
        MagblockServoSubsystem.block()

        odom = OdometrySubsystem(72.0, 72.0, -PI / 2, hardwareMap)
    }

    private var autoAimEnabled = true;
    private var resetMode = false;
    private var resetModePhiAngle = 180.0.deg;
    private var phiTrim = 0.0.deg;
    var speedFactor = 1.0;
    override fun onStartButtonPressed() {
        gamepad1.setLedColor(0.0, 0.0, 255.0, -1)
        gamepad2.setLedColor(255.0, 0.0, 0.0, -1)

        val file = File(Lefile.filePath)
        val content = file.readText().split("\n")
        val startX = content[0].toDouble()
        val startY = content[1].toDouble()
        val startH = content[2].toDouble()

        odom!!.setPinpoint(startX, startY, startH)
//        odom!!.setPinpoint(72.0, 72.0, -PI / 2)

        val mecanum = MecanumDriverControlled(
            lfw,
            rfw,
            lbw,
            rbw,
            -Gamepads.gamepad1.leftStickY.map { it*speedFactor },
            Gamepads.gamepad1.leftStickX.map { it*speedFactor },
            Gamepads.gamepad1.rightStickX.map { it*speedFactor },
            FieldCentric {
                if (isBlue) (h.inRad - PI).rad else h
            }
        )
        mecanum();

        Gamepads.gamepad1.rightBumper whenBecomesTrue {
            speedFactor = 0.5;
        } whenBecomesFalse {
            speedFactor = 1.0;
        }

        val lowerMotorDrive = MagMotorSubsystem.DriverCommand(
            Gamepads.gamepad2.rightTrigger,
            Gamepads.gamepad2.leftTrigger
        );
        lowerMotorDrive();

        val intakeMotorDrive = IntakeMotorSubsystem.DriverCommand(
            Gamepads.gamepad2.rightTrigger,
            Gamepads.gamepad2.leftTrigger
        )
        intakeMotorDrive()

//        val magServoDrive = MagServoSubsystem.DriverCommandDefaultOn(
//            Gamepads.gamepad1.leftTrigger.greaterThan(0.0)
//        )
//        magServoDrive();
        Gamepads.gamepad1.leftTrigger.greaterThan(0.0) whenBecomesTrue MagServoSubsystem.reverse
        Gamepads.gamepad1.rightTrigger.greaterThan(0.0) whenBecomesTrue MagServoSubsystem.run

        Gamepads.gamepad1.circle whenBecomesTrue ParallelGroup(
            MagServoSubsystem.run,
            MagblockServoSubsystem.unblock
        ) whenBecomesFalse ParallelGroup(
            MagServoSubsystem.stop,
            MagblockServoSubsystem.block
        )

        // manual mode toggle
        Gamepads.gamepad2.leftBumper and Gamepads.gamepad2.triangle whenBecomesTrue {
            autoAimEnabled = !autoAimEnabled;
            gamepad1.rumble(450);
            gamepad2.rumble(450);
        }

        // reset mode toggle
        Gamepads.gamepad2.leftBumper and Gamepads.gamepad2.rightBumper whenBecomesTrue {
            resetMode = !resetMode;
            if (resetMode) {
                // 180.0.deg corresponds to turret facing backwards
                resetModePhiAngle = 180.0.deg
                gamepad2.rumble(200)
                gamepad2.setLedColor(100.0, 0.0, 0.0, -1)
            } else {
                // reset position
                ofsX = resetModeParams.x - x
                ofsY = resetModeParams.y - y
                ofsH = resetModeParams.h.inRad - h.inRad
                gamepad2.rumble(200)
                gamepad2.setLedColor(255.0, 0.0, 0.0, -1)
            }
        }
        // I think l/r only makes sense when robot facing away (approx same direction person is facing)
        Gamepads.gamepad2.dpadRight whenBecomesTrue {
            phiTrim -= 1.0.deg
        }
        Gamepads.gamepad2.dpadLeft whenBecomesTrue {
            phiTrim += 1.0.deg
        }
    }

    var lastRuntime = 0.0
    override fun onUpdate() {
        telemetry.addData("Loop Time (ms)", runtime - lastRuntime);
        lastRuntime = runtime;

        odom!!.updateOdom()

        val dx = goalX - x
        val dy = goalY - y
        val dxy = hypot(dx, dy)
        val dxp = dx + vx * distanceToTime(dxy)
        val dyp = dy + vy * distanceToTime(dxy)
        val dxyp = hypot(dxp, dyp)
        val hp = h + vh * distanceToTime(dxyp)

        if (resetMode) {
            TurretPhiSubsystem.SetTargetPhi(resetModePhiAngle).requires(TurretPhiSubsystem)()
        } else if (autoAimEnabled) {
            ShooterSubsystem.AutoAim(
                dxyp,
                distanceToVelocity
            )()
            TurretPhiSubsystem.AutoAim(
                dxp, dyp, hp, phiTrim
            )()
            TurretThetaSubsystem.AutoAim(
                dxyp,
                distanceToTheta
            )()
        } else {
            //ShooterSubsystem.Manual(

            //)
        }

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

//        PanelsTelemetry.telemetry.addData("Vx (in/s)", vx)
//        PanelsTelemetry.telemetry.addData("Vy (in/s)", vy)
        PanelsTelemetry.telemetry.addData("CMD", CommandManager.snapshot)
        PanelsTelemetry.telemetry.update()
    }

    override fun onStop() {
        val file = File(Lefile.filePath)
        file.writeText(
            x.toString() + "\n" +
                    y.toString() + "\n" +
                    h.inRad.toString() + "\n"
        )
    }
}