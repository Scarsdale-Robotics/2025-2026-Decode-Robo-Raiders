package org.firstinspires.ftc.teamcode.subsystems.lower.magazine

import com.acmerobotics.dashboard.config.Config
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition

@Config
object MagblockServoSubsystem : Subsystem {
    @JvmField var OPEN = 1.0;
    @JvmField var CLOSED = 0.0;

    private val servo = ServoEx("magblock");

    @JvmField val open = InstantCommand { SetPosition(servo, OPEN); }
    val close = InstantCommand { SetPosition(servo, CLOSED); }
}
