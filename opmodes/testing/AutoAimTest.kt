package org.firstinspires.ftc.teamcode.opmodes.testing

import com.acmerobotics.dashboard.config.Config
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.deg
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem

@Config
class AutoAimTest : NextFTCOpMode() {
    @JvmField var shootAngleDegrees = 45.0;

    init {
        addComponents(
            SubsystemComponent(
                OuttakeSubsystem
            )
        );
    }

    override fun onStartButtonPressed() {
        ShooterSubsystem.Run().schedule();
    }

    override fun onUpdate() {
        TurretThetaSubsystem.targetTheta = shootAngleDegrees.deg;
    }
}
