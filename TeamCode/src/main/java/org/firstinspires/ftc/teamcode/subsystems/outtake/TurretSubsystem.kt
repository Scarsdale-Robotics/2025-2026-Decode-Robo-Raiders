package org.firstinspires.ftc.teamcode.subsystems.outtake

import dev.nextftc.bindings.Button
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.SubsystemGroup
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.deg
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem.manualAimPhiChangeDegsPerSecond
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem.targetPhi
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem.manualAimThetaChangeDegsPerSecond
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem.targetTheta
import java.util.function.Supplier
import kotlin.math.hypot
import kotlin.time.ComparableTimeMark
import kotlin.time.DurationUnit
import kotlin.time.TimeSource

object TurretSubsystem : SubsystemGroup(
    TurretPhiSubsystem,
    TurretThetaSubsystem
) {
    class AutoAim(
        private val dx: Supplier<Double>,  // distance from turret to goal in x axis; positive means goal is right of turret
        private val dy: Supplier<Double>,
        angleByDistance: (Double) -> Angle,
    ) : ParallelGroup(
        TurretPhiSubsystem.AutoAim(dx, dy),
        TurretThetaSubsystem.AutoAim({ hypot(dx.get(), dy.get()) }, angleByDistance),
    )

    open class DriverCommand @JvmOverloads constructor(
        private val farModeButton: Button,
        private val nearModeButton: Button,
        private val shiftUpButton: Button,
        private val shiftDownButton: Button,
        private val shiftLeftButton: Button,
        private val shiftRightButton: Button,
        private val setButton: Button,
        initialFarModeTheta: Angle,
        initialNearModeTheta: Angle,
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

        var farModeTheta = initialFarModeTheta;
        var nearModeTheta = initialNearModeTheta;
        var farModePhi = initialFarModePhi;
        var nearModePhi = initialNearModePhi;

        var lastTimestamp: ComparableTimeMark? = null;

        override fun update() {
            val timestamp = timeSource.markNow();
            val dt = (timestamp - (lastTimestamp?:timestamp))
                .toDouble(DurationUnit.SECONDS);
            lastTimestamp = timestamp;

            // manual move
            shiftUpButton whenBecomesTrue TurretThetaSubsystem.SetTargetTheta(
                targetTheta + manualAimThetaChangeDegsPerSecond.deg * dt
            )
            shiftDownButton whenBecomesTrue TurretThetaSubsystem.SetTargetTheta(
                targetTheta - manualAimThetaChangeDegsPerSecond.deg * dt
            )
            shiftLeftButton whenBecomesTrue TurretPhiSubsystem.SetTargetPhi(
                targetPhi - manualAimPhiChangeDegsPerSecond.deg * dt
            )
            shiftRightButton whenBecomesTrue TurretPhiSubsystem.SetTargetPhi(
                targetPhi + manualAimPhiChangeDegsPerSecond.deg * dt
            )

            // move to template locations
            farModeButton whenBecomesTrue ParallelGroup(
                TurretThetaSubsystem.SetTargetTheta(farModeTheta),
                TurretPhiSubsystem.SetTargetPhi(farModePhi),
                InstantCommand { mode = DistanceMode.FAR }
            )
            nearModeButton whenBecomesTrue ParallelGroup(
                TurretThetaSubsystem.SetTargetTheta(nearModeTheta),
                TurretPhiSubsystem.SetTargetPhi(nearModePhi),
                InstantCommand { mode = DistanceMode.CLOSE }
            )

            // set template locations
            setButton whenBecomesTrue {
                when (mode) {
                    DistanceMode.FAR -> {
                        farModeTheta = targetPhi;
                        farModePhi = targetPhi;
                    }
                    DistanceMode.CLOSE -> {
                        nearModeTheta = targetPhi;
                        nearModePhi = targetPhi;
                    }
                }
            }
        }

        override fun stop(interrupted: Boolean) {
            lastTimestamp = null;
        }
    }
}
