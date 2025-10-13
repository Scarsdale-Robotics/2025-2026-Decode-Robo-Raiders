package org.firstinspires.ftc.teamcode.subsystems.outtake.turret

import androidx.core.math.MathUtils
import com.acmerobotics.dashboard.config.Config
import dev.nextftc.control.ControlSystem
import dev.nextftc.control.KineticState
import dev.nextftc.control.feedback.FeedbackType
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.rad
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.utils.SMO.SMOFilter
import java.util.function.Supplier
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

// left-right
@Config
object TurretPhiSubsystem : Subsystem {
    private val motor = MotorEx("turret_phi");

    @JvmField val ENCODERS_FORWARD = 0.0;
    @JvmField val ENCODERS_BACKWARD = 3000.0;  // todo: TUNE

    private val controlSystem: ControlSystem;

    @JvmField var kSqu = 0.0;
    @JvmField var kI = 0.0;
    @JvmField var kD = 0.0;

    @JvmField var Ls = 0.0;
    @JvmField var Lv = 0.0;

    init {
        val posSMO = SMOFilter(FeedbackType.POSITION, Lv, Ls);

        controlSystem = ControlSystem()
            .posFilter { filter -> filter.custom(posSMO).build(); }
            .posSquID(PIDCoefficients(kSqu, kI, kD))
            .build();
    }

    var targetPhi: Angle = 0.0.rad
        set(value) {
            val norm = atan2(sin(value.inRad), cos(value.inRad)).rad;
            field = norm;
            controlSystem.goal = KineticState(
                norm / kotlin.math.PI.rad *
                        (ENCODERS_FORWARD - ENCODERS_BACKWARD) + ENCODERS_FORWARD
            );
        }

    class AutoAim(
        private val dx: Supplier<Double>,
        private val dy: Supplier<Double>,
    ) : Command() {
        override val isDone = false;

        override fun update() {
            targetPhi = atan2(dy.get(), dx.get()).rad;
        }
    }

    class DriverCommand(

    ) // todo

    override fun periodic() {
        motor.power = controlSystem.calculate(motor.state);
    }

}