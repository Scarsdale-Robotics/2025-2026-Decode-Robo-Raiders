package org.firstinspires.ftc.teamcode.subsystems.outtake.turret

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
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
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.time.ComparableTimeMark
import kotlin.time.DurationUnit
import kotlin.time.TimeSource

// left-right
@Configurable
object TurretPhiSubsystem : Subsystem {
    private val motor = MotorEx("turret_phi");

    @JvmField var ENCODERS_FORWARD = 1367.0;
    @JvmField var ENCODERS_BACKWARD = 0.0;  // todo: TUNE

    private val controller: ControlSystem;

    @JvmField var squidCoefficients = PIDCoefficients(0.001, 0.0, 0.0);

//    @JvmField var Ls = 0.0;
//    @JvmField var Lv = 0.0;

    init {
//        val posSMO = SMOFilter(FeedbackType.POSITION, Lv, Ls);

        controller = ControlSystem()
//            .posFilter { filter -> filter.custom(posSMO).build(); }
            .posSquID(squidCoefficients)
            .build();

        controller.goal = KineticState()
    }

    override fun initialize() {
    }

    fun reset() {
        var d = ENCODERS_FORWARD - ENCODERS_BACKWARD
        ENCODERS_FORWARD = motor.currentPosition
        ENCODERS_BACKWARD = ENCODERS_FORWARD - d
    }

    // 0.0 --> robot forward
    var targetPhi: Angle = 0.0.rad
        get() {
            return norm(
                PI.rad * (motor.currentPosition - ENCODERS_FORWARD) /
                        (ENCODERS_FORWARD - ENCODERS_BACKWARD)
            )
        }
        private set;

    fun norm(angle: Angle): Angle {
//        return atan2(sin(angle.inRad), cos(angle.inRad)).rad;
        val tolerance = PI / 6;
        var a = angle.inRad;

        if (a < -2 * PI -1.0/12.0 - tolerance) {
            a += 2 * PI;
        } else if (a > -1.0/12.0 + tolerance) {
            a -= 2 * PI;
        }
        return a.rad
    }

    open class SetTargetPhi(val angle: Angle, ofsTurret: Double = 0.0) : RunToState(
        controller,
        KineticState(
            norm(angle) / PI.rad *
            (ENCODERS_FORWARD - ENCODERS_BACKWARD) + ENCODERS_FORWARD
            + ofsTurret
        )
    )

    var lastCommand: Command? = null;

    class AutoAim(
        private val dx: Double,
        private val dy: Double,
        private val rh: Angle,
        private val ofsTurret: Double = 0.0
    ) : Command() {
        override val isDone = true;

        init {
            setName("Auto Aim Phi")
        }

        override fun update() {
            if (lastCommand != null) {
                CommandManager.cancelCommand(lastCommand!!)
            }
            val currCommand = SetTargetPhi(atan2(dy, dx).rad - rh, ofsTurret);
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
        val power = controller.calculate(
            motor.state
        )
        SetPower(motor, power).setInterruptible(true)()

        PanelsTelemetry.telemetry.addData("phi enc", motor.currentPosition)
        PanelsTelemetry.telemetry.addData("ref", controller.reference)
        PanelsTelemetry.telemetry.addData("goal", controller.goal)
        PanelsTelemetry.telemetry.addData("turret phi", targetPhi)
        PanelsTelemetry.telemetry.addData("lm", controller.lastMeasurement)
    }

}