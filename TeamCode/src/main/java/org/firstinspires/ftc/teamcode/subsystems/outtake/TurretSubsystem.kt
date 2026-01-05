package org.firstinspires.ftc.teamcode.subsystems.outtake

import dev.nextftc.bindings.Button
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.SubsystemGroup
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.deg
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem
//import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem.manualAimPhiChangeDegsPerSecond
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem.targetPhi
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem
//import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem.manualAimThetaChangeDegsPerSecond
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
        rh: Supplier<Angle>,
        angleByDistance: (Double) -> Angle,
    ) : ParallelGroup(
        TurretPhiSubsystem.AutoAim(dx, dy, rh),
        TurretThetaSubsystem.AutoAim({ hypot(dx.get(), dy.get()) }, angleByDistance),
    )

    class Manual(
        goalChangePhi: Supplier<Double>,
        goalChangeTheta: Supplier<Double>
    ) : ParallelGroup(
        TurretThetaSubsystem.Manual(goalChangeTheta),
        TurretPhiSubsystem.Manual(goalChangePhi)
    )
}
