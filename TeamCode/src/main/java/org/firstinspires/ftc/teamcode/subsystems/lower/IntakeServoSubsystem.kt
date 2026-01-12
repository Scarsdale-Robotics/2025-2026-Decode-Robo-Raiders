package org.firstinspires.ftc.teamcode.subsystems.lower

import com.acmerobotics.dashboard.config.Config
import com.bylazar.configurables.annotations.Configurable
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition


@Configurable
object IntakeServoSubsystem : Subsystem {
    @JvmField var DOWN = 0.0;  // todo: tune
    @JvmField var UP = 0.3;

    private val servo = ServoEx("intake_servo");

    val up = SetPosition(servo, UP)
    val down = SetPosition(servo, DOWN)
}
