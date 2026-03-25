package org.firstinspires.ftc.teamcode.subsystems.outtake.turret
import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.rad
import java.util.function.Supplier
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import dev.nextftc.hardware.impl.ServoEx

//IN CONFIG, SET SERVO 1 TO 0 AND SERVO 2 TO 0.99
@Configurable
object TurretPhiSubsystem : Subsystem {
    private val servoBelow = ServoEx("turret_below", 0.0001);
    private val servoAbove = ServoEx("turret_above", 0.0001);
    val MIN_ANGLE = Math.toRadians(-360.0)
    val MAX_ANGLE = Math.toRadians(6.7)
    var targetPhi: Angle = 0.0.rad
    var lmao = 0.98375  // servo range to remain flat

    var started = false;

    init {
//
    }

    fun zero() {}

    override fun initialize() {
        started = true;
    }
    fun angleToServo(angle: Angle): Double {
        val a = angle.inRad.coerceIn(MIN_ANGLE, MAX_ANGLE)
        return (a - MIN_ANGLE) / (MAX_ANGLE - MIN_ANGLE) * lmao
    }


    fun norm(angle: Angle): Angle {
        return angle.inRad.coerceIn(MIN_ANGLE, MAX_ANGLE).rad;

//        return atan2(sin(angle.inRad), cos(angle.inRad)).rad;
//        val tolerance = Math.toRadians(10.0);
//        val dzHalf = Math.toRadians(19.0);
//        var a = angle.inRad;
//
//        val LOWER = Math.toRadians(51.0) - 2*PI;
//        val UPPER = Math.toRadians(51.0);
//
//        while (a < LOWER - tolerance) {
//            a += 2 * PI;
//        }
//        while (a > UPPER + tolerance) {
//            a -= 2 * PI;
//        }
//        return a.coerceIn(LOWER + dzHalf, UPPER - dzHalf).rad;
//        Math.max(Math.min(a, UPPER), LOWER).rad;

//        while (a < 0.27 - 2 * PI - tolerance) {
//            a += 2 * PI;
//        }
//        while (a > 0.27 + tolerance) {
//            a -= 2 * PI;
//        }
//        return a.rad;
//        return max(min(a, PI / 4.0), -7.0 * PI / 4.0).rad
    }

    class SetTargetPhi(
        private val angle: Angle,
        private val ofsTurret: Angle = 0.0.rad
    ): Command() {
        override val isDone = true;

        init {
            requires(TurretPhiSubsystem)
        }

        override fun start() {
            val normed = norm(angle + ofsTurret)
            val pos = angleToServo(normed)

            servoBelow.position = pos
            servoAbove.position = lmao - pos

            targetPhi = normed
        }
    }

    var lastCommand: Command? = null;

    //i have no idea what this means nathan
    class AutoAim(
        private val dx: Double,
        private val dy: Double,
        private val rh: Angle,
        private val ofsTurret: Angle = 0.0.rad,
    ) : Command() {
        override val isDone = true;

        init {
            setName("Auto Aim Phi")
        }

        override fun start() {
            if (lastCommand != null) {
                CommandManager.cancelCommand(lastCommand!!)
            }

            val normAngle = (atan2(dy, dx).rad - rh).inRad
            val upper = normAngle + 2 * PI;
            val lower = normAngle - 2 * PI;
            var closestDist = abs(normAngle - targetPhi.inRad)
            val upperDist = abs(upper - targetPhi.inRad)
            val lowerDist = abs(lower - targetPhi.inRad)
            var closest = normAngle;
            if (upperDist < closestDist) {
                closest = upper
                closestDist = upperDist
            }
            if (lowerDist < closestDist) {
                closest = lower
            }

            SetTargetPhi(closest.rad, ofsTurret)()
        }
    }

    @JvmField var thing = 3.0;
    class Manual(
        private val goalChange: Supplier<Double>
    ) : Command() {

        override val isDone = false

        override fun update() {
            val delta = goalChange.get() * 0.01

            val newPos = (servoBelow.position + delta).coerceIn(0.0, 1.0)

            servoBelow.position = newPos
            servoAbove.position = lmao - newPos
        }
    }

    override fun periodic() {
        if (!started) return;
        PanelsTelemetry.telemetry.addData("target phi", targetPhi)
        PanelsTelemetry.telemetry.addData("servo1 pos", servoBelow.position)
    }
}