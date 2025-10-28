package org.firstinspires.ftc.teamcode.subsystems.lower.magazine

import com.acmerobotics.dashboard.config.Config
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower

@Config
object MagazineMotorSubsystem : Subsystem {
    @JvmField var FORWARD = 1.0;
    @JvmField var REVERSE = -1.0;

    private val motor = MotorEx("magazine");

    @JvmStatic val forward = SetPower(motor, FORWARD);
    @JvmStatic val reverse = SetPower(motor, REVERSE);
}
