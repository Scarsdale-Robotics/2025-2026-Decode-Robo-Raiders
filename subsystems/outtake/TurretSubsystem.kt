package org.firstinspires.ftc.teamcode.subsystems.outtake

import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.subsystems.SubsystemGroup
import dev.nextftc.core.units.Angle
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem
import java.util.function.Supplier
import kotlin.math.hypot

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

    class DriverCommand() : ParallelGroup(
        TurretPhiSubsystem.DriverCommand(),
        TurretThetaSubsystem.DriverCommand()
    )  // todo
}