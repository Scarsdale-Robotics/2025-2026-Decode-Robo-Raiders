package org.firstinspires.ftc.teamcode.subsystems.outtake.turret

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import dev.nextftc.bindings.Button
import dev.nextftc.core.commands.Command
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition
import dev.nextftc.hardware.positionable.SetPositions
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem.targetPhi
import java.util.function.Supplier
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.time.ComparableTimeMark
import kotlin.time.DurationUnit
import kotlin.time.TimeSource

// up-down
@Configurable
// to reset servo, place at pos zero, then lay hood on, then inc servo value
object TurretThetaSubsystem : Subsystem {
    private val servo = ServoEx("turret_theta");

    @JvmField var POS_63deg = 0.58;
    @JvmField var POS_55deg = 0.31;

    val open = SetPosition(servo, 0.1).requires(this)

    var targetTheta: Angle = 0.0.rad
        get() {
            return norm(
                8.0.deg *
                        ((servo.position - POS_55deg) / (POS_63deg - POS_55deg))
                        + 55.0.deg
            )
        }
        private set

    fun norm(angle: Angle): Angle {
        return atan2(sin(angle.inRad), cos(angle.inRad)).rad;
    }

    class SetTargetTheta(val angle: Angle) : SetPositions(
        servo to ((norm(angle) - 55.0.deg) / 8.0.deg * (POS_63deg - POS_55deg) + POS_55deg)
    )

    class AutoAim(
        private val dxy: Supplier<Double>,
        private val angleByDistance: (Double) -> Angle,  // get by running curve of best fit on collected data
    ) : Command() {
        override val isDone = false;

        init {
            setName("Auto Aim Theta")
        }

        override fun start() {
            SetTargetTheta(55.0.deg);
        }
        
        override fun update() {
            SetTargetTheta(angleByDistance(dxy.get()))();
            PanelsTelemetry.telemetry.addData("s", dxy.get())
            PanelsTelemetry.telemetry.addData("theta goal", angleByDistance(dxy.get()))
        }
    }

    @JvmField var manualAimThetaChangeDegsPerSecond = 0.5;

    class DriverCommand(
        private val farModeButton: Button,
        private val nearModeButton: Button,
        private val shiftUpButton: Button,
        private val shiftDownButton: Button,
        private val setButton: Button,
        initialFarModeTheta: Angle,
        initialNearModeTheta: Angle,
        private val timeSource: TimeSource.WithComparableMarks = TimeSource.Monotonic,
    ) : Command() {
        enum class DistanceMode {
            FAR,
            CLOSE
        }

        override val isDone = false;

        var mode = DistanceMode.FAR;

        var farModeTheta = initialFarModeTheta;
        var nearModeTheta = initialNearModeTheta;

        var lastTimestamp: ComparableTimeMark? = null;

        override fun update() {
            val timestamp = timeSource.markNow();
            val dt = (timestamp - (lastTimestamp?:timestamp))
                .toDouble(DurationUnit.SECONDS);
            lastTimestamp = timestamp;

            // manual move
            shiftUpButton whenBecomesTrue {
                targetTheta += manualAimThetaChangeDegsPerSecond.deg * dt;
            }
            shiftDownButton whenBecomesTrue {
                targetTheta -= manualAimThetaChangeDegsPerSecond.deg * dt;
            }

            // move to template locations
            farModeButton whenBecomesTrue {
                mode = DistanceMode.FAR;
                targetTheta = farModeTheta;
            }
            nearModeButton whenBecomesTrue {
                mode = DistanceMode.CLOSE;
                targetTheta = nearModeTheta;
            }

            // set template locations
            setButton whenBecomesTrue {
                when (mode) {
                    DistanceMode.FAR -> farModeTheta = targetPhi;
                    DistanceMode.CLOSE -> nearModeTheta = targetPhi;
                }
            }
        }

        override fun stop(interrupted: Boolean) {
            lastTimestamp = null;
        }
    }
}