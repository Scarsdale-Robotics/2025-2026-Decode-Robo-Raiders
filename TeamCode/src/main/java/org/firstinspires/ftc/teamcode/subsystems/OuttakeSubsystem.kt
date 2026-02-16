package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.telemetry.PanelsTelemetry
import dev.nextftc.core.commands.Command
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
        private val dx: Double,  // distance from turret to goal in x axis; positive means goal is right of turret
        private val dy: Double,
        private val rh: Angle,
        private val veloByDistance: (Double) -> Double,
        private val angleByDistanceAndVel: (Double, Double) -> Angle,  // distance, velocity (dist first)
    ) : Command() {
        override val isDone = true;

        init {
            setName("Auto Aim Outtake")
        }

        override fun start() {
            val dxy = hypot(dx, dy)
            ShooterSubsystem.AutoAim(dxy, veloByDistance)
            TurretPhiSubsystem.AutoAim(dx, dy, rh);
            TurretThetaSubsystem.SetTargetTheta(angleByDistanceAndVel(dxy, ShooterSubsystem.velocity))();
        }
    }
//
//    class Manual(
//        goalChangePhi: Supplier<Double>,
//        goalChangeTheta: Supplier<Double>,
//        shooterPower: Supplier<Double>
//    ) : ParallelGroup(
//        TurretThetaSubsystem.Manual(goalChangeTheta),
//        TurretPhiSubsystem.Manual(goalChangePhi),
//        ShooterSubsystem.Manual(shooterPower)
//    )
}
