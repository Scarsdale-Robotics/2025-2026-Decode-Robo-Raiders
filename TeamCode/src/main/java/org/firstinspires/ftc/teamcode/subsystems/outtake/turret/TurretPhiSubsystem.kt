package org.firstinspires.ftc.teamcode.subsystems.outtake.turret

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import dev.nextftc.bindings.Button
import dev.nextftc.control.ControlSystem
import dev.nextftc.control.KineticState
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.hardware.controllable.RunToState
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem.setMotorPowers
import org.opencv.video.BackgroundSubtractor
import java.util.function.Supplier
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin
import kotlin.time.ComparableTimeMark
import kotlin.time.DurationUnit
import kotlin.time.TimeSource
import dev.nextftc.hardware.impl.ServoEx

// left-right
@Configurable
object TurretPhiSubsystem : Subsystem {
    private val servo1 = ServoEx("servo_one");
    private val servo2 = ServoEx("servo_two")

    var started = false;

    init {
//
    }

    override fun initialize() {
        started = true;
    }

    // 0.0 --> robot forward
    //var targetPhi: Angle = 0.0.rad
      //  get() {
        //    return PI.rad * (motor.currentPosition - ENCODERS_FORWARD) /
          //          (ENCODERS_FORWARD - ENCODERS_BACKWARD)
        //}
        //private set;

    fun norm(angle: Angle): Angle {
//        return atan2(sin(angle.inRad), cos(angle.inRad)).rad;
        val tolerance = Math.toRadians(10.0);
        val dzHalf = Math.toRadians(19.0);
        var a = angle.inRad;

        val LOWER = Math.toRadians(51.0) - 2*PI;
        val UPPER = Math.toRadians(51.0);

        while (a < LOWER - tolerance) {
            a += 2 * PI;
        }
        while (a > UPPER + tolerance) {
            a -= 2 * PI;
        }
        return a.coerceIn(LOWER + dzHalf, UPPER - dzHalf).rad;
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

    open class SetTargetPhi(val angle: Angle, ofsTurret: Angle = 0.0.rad) : RunToState(
        controller,
        KineticState(
            (norm(angle + ofsTurret)) / PI.rad *
                    (ENCODERS_FORWARD - ENCODERS_BACKWARD) + ENCODERS_FORWARD
        )
    )

    var lastCommand: Command? = null;

    class AutoAim(
        private val dx: Double,
        private val dy: Double,
        private val rh: Angle,
        private val ofsTurret: Angle = 0.0.rad,
        private val feedforward: Double = 0.0
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

            val currCommand = SetTargetPhi(closest.rad, ofsTurret);
            currCommand();
            lastCommand = currCommand;
        }
    }

    @JvmField var thing = 3.0;
    class Manual(
        private val goalChange: Supplier<Double>
    ) : Command() {
        override val isDone = false;

        override fun update() {
            RunToState(
                controller,
                KineticState(
                    controller.goal.position + goalChange.get() * thing
                )
            )
        }
    }

    override fun periodic() {
        if (!started) return;
        secondaryController.goal = controller.goal
//        tertiaryController.goal = controller.goal
        var power = controller.calculate(
            motor.state
        )
        val error = abs(controller.goal.position - controller.lastMeasurement.position)
        if (error < 6.7) {
            power = 0.0
//        } else if (error < 50.22) {
//            power = tertiaryController.calculate(
//                motor.state
//            )
//
//            // add feedforward
//            power += feedforwardCmd * feedforwardCoefficient;
        } else if (error < 111.111) {
            power = secondaryController.calculate(
                motor.state
            )

            // add feedforward
            power += feedforwardCmd * feedforwardCoefficient;
        }
        SetPower(motor, power).setInterruptible(true)()

        PanelsTelemetry.telemetry.addData("phi enc", motor.currentPosition)
        PanelsTelemetry.telemetry.addData("ref", controller.reference)
        PanelsTelemetry.telemetry.addData("goal", controller.goal)
        PanelsTelemetry.telemetry.addData("turret phi", targetPhi)
        PanelsTelemetry.telemetry.addData("lm", controller.lastMeasurement)
    }


}