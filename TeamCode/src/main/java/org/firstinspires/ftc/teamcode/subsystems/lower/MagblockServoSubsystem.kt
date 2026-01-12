package org.firstinspires.ftc.teamcode.subsystems.lower

import com.bylazar.configurables.annotations.Configurable
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition

@Configurable
object MagblockServoSubsystem : Subsystem {
    @JvmField var UNBLOCK: Double? = 0.3;  // todo: tune
    @JvmField var BLOCK: Double? = 0.0;

    private val servo = ServoEx("magblock");

    // using functions instead of fields makes the opmodes much cleaner: you don't need new each time, but the parentheses are still there to give meathod-vibes
    val unblock = SetPosition(servo, UNBLOCK!!)
    val block = SetPosition(servo, BLOCK!!)
}
