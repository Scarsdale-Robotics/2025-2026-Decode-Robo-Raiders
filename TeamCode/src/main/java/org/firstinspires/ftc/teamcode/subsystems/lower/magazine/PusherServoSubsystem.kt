package org.firstinspires.ftc.teamcode.subsystems.lower.magazine

import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition

object PusherServoSubsystem : Subsystem {
    @JvmField var OUT = 0.5;  // TODO: set
    @JvmField var IN = 0.0;

    private val servo = ServoEx("pusher");

    val push = SequentialGroup(
        SetPosition(servo, OUT),
        SetPosition(servo, IN)
    ).requires(this);
}