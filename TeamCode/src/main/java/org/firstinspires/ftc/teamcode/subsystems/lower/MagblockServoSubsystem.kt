package org.firstinspires.ftc.teamcode.subsystems.lower

import com.bylazar.configurables.annotations.Configurable
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition

@Configurable
object MagblockServoSubsystem : Subsystem {
    @JvmField var OPEN: Double? = 0.98;
    @JvmField var CLOSED: Double? = 0.76;

    private val servo = ServoEx("magblock");

    // using functions instead of fields makes the opmodes much cleaner: you don't need new each time, but the parentheses are still there to give meathod-vibes
    val open = SetPosition(servo, OPEN!!)
    val close = SetPosition(servo, CLOSED!!)
}
