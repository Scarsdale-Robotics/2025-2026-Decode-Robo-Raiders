package org.firstinspires.ftc.teamcode.subsystems.outtake.turret

import com.acmerobotics.dashboard.config.Config
import dev.nextftc.bindings.Button
import dev.nextftc.core.commands.Command
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem.targetPhi
import java.util.function.Supplier
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin
import kotlin.time.ComparableTimeMark
import kotlin.time.DurationUnit
import kotlin.time.TimeSource

// up-down
@Config
object TurretThetaSubsystem : Subsystem {
    private val servo = ServoEx("turret_theta");

    @JvmField var POS_58deg = 0.0;
    @JvmField var POS_49deg = 0.5;  // todo: TUNE

    var targetTheta: Angle = 0.0.rad
        set(value) {
            val norm = atan2(sin(value.inRad), cos(value.inRad)).rad;
            field = norm;
            SetPosition(
                servo,
                (norm - 49.0.deg) / 9.0.deg *
                        (POS_58deg - POS_49deg) + POS_49deg
            );
        }

    class AutoAim(
        private val dxy: Supplier<Double>,
        private val angleByDistance: (Double) -> Angle,  // get by running curve of best fit on collected data
    ) : Command() {
        override val isDone = false;

        override fun update() {
            targetTheta = angleByDistance(dxy.get());
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