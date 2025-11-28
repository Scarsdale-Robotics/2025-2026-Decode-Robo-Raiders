package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.subsystems.SubsystemGroup
import dev.nextftc.core.units.Angle
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.TurretSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem
import java.util.function.Supplier
import kotlin.math.hypot

object OuttakeSubsystem : SubsystemGroup(
    ShooterSubsystem,
    TurretSubsystem,
) {
    class AutoAim(
        private val dx: Supplier<Double>,  // distance from turret to goal in x axis; positive means goal is right of turret
        private val dy: Supplier<Double>,
        rh: Supplier<Angle>,
        angleByDistance: (Double) -> Angle,
        powerByDistance: (Double) -> Double,
    ) : ParallelGroup(
        TurretPhiSubsystem.AutoAim(dx, dy, rh),
        TurretThetaSubsystem.AutoAim({ hypot(dx.get(), dy.get()) }, angleByDistance),
        ShooterSubsystem.AutoAim({ hypot(dx.get(), dy.get()) }, powerByDistance),
    )
}
