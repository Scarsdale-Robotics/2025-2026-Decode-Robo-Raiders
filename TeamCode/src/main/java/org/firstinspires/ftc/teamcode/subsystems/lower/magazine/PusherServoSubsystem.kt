package org.firstinspires.ftc.teamcode.subsystems.lower.magazine

import com.bylazar.configurables.annotations.Configurable
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition
import kotlin.time.Duration.Companion.milliseconds

@Configurable
object PusherServoSubsystem : Subsystem {
    @JvmField var OUT = 0.34;
    @JvmField var IN = 0.08;
    @JvmField var DELAY_MS_PUSH = 1000;

    private val servo = ServoEx("magpush");

    val push = SequentialGroup(
        SetPosition(servo, IN),
        Delay(DELAY_MS_PUSH.milliseconds),
        SetPosition(servo, OUT)
    ).requires(this);

    val out = SetPosition(servo, OUT);
    val `in` = SetPosition(servo, IN);
}