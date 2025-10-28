package org.firstinspires.ftc.teamcode.subsystems.outtake.turret

import com.acmerobotics.dashboard.config.Config
import dev.nextftc.bindings.Button
import dev.nextftc.control.ControlSystem
import dev.nextftc.control.KineticState
import dev.nextftc.control.feedback.FeedbackType
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.core.commands.Command
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower
import org.firstinspires.ftc.teamcode.utils.SMO.SMOFilter
import java.util.function.Supplier
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.time.ComparableTimeMark
import kotlin.time.DurationUnit
import kotlin.time.TimeSource

// left-right
@Config
object TurretPhiSubsystem : Subsystem {
    private val motor = MotorEx("turret_phi");

    @JvmField var ENCODERS_FORWARD = 0.0;
    @JvmField var ENCODERS_BACKWARD = 3000.0;  // todo: TUNE

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

    @JvmField var manualAimPhiChangeDegsPerSecond = 3;

    class DriverCommand(
        private val farModeButton: Button,
        private val nearModeButton: Button,
        private val shiftLeftButton: Button,
        private val shiftRightButton: Button,
        private val setButton: Button,
        initialFarModePhi: Angle,
        initialNearModePhi: Angle,
        private val timeSource: TimeSource.WithComparableMarks = TimeSource.Monotonic,
    ) : Command() {
        enum class DistanceMode {
            FAR,
            CLOSE
        }

        override val isDone = false;

        var mode = DistanceMode.FAR;

        var farModePhi = initialFarModePhi;
        var nearModePhi = initialNearModePhi;

        var lastTimestamp: ComparableTimeMark? = null;

        override fun update() {
            val timestamp = timeSource.markNow();
            val dt = (timestamp - (lastTimestamp?:timestamp)).toDouble(DurationUnit.SECONDS);
            lastTimestamp = timestamp;

            // manual move
            shiftLeftButton whenBecomesTrue {
                targetPhi -= manualAimPhiChangeDegsPerSecond.deg * dt;
            }
            shiftRightButton whenBecomesTrue {
                targetPhi += manualAimPhiChangeDegsPerSecond.deg * dt;
            }

            // move to template locations
            farModeButton whenBecomesTrue {
                mode = DistanceMode.FAR;
                targetPhi = farModePhi;
            }
            nearModeButton whenBecomesTrue {
                mode = DistanceMode.CLOSE;
                targetPhi = nearModePhi;
            }

            // set template locations
            setButton whenBecomesTrue {
                when (mode) {
                    DistanceMode.FAR -> farModePhi = targetPhi;
                    DistanceMode.CLOSE -> nearModePhi = targetPhi;
                }
            }
        }

        override fun stop(interrupted: Boolean) {
            lastTimestamp = null;
        }
    }

    override fun periodic() {
        SetPower(motor, controlSystem.calculate(motor.state));
    }

}