package org.firstinspires.ftc.teamcode.opmodes.testing

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition
import dev.nextftc.hardware.powerable.SetPower

@TeleOp(name = "Magblock Test", group = "Config")
@Configurable
class MagblockTest : NextFTCOpMode() {
    val servo = ServoEx("magblock")

    companion object {
        var position = 0.0;
    }

    override fun onUpdate() {
        SetPosition(servo, position)();
    }
}