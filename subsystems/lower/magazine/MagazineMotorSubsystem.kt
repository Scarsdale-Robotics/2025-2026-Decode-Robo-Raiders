package org.firstinspires.ftc.teamcode.subsystems.lower.magazine

import com.acmerobotics.dashboard.config.Config
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower

@Config
object MagazineMotorSubsystem : Subsystem {
    @JvmField var POWER_FORWARD = 1.0;
    @JvmField var POWER_REVERSE = -1.0;

    private val motor = MotorEx("magazine");

    private var power = 0.0;

    val forward = InstantCommand { power = POWER_FORWARD };
    val reverse = InstantCommand { power = POWER_REVERSE };

    override fun periodic() {
        SetPower(motor, power);
    }
}