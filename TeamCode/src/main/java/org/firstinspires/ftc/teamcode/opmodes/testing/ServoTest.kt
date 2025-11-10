package org.firstinspires.ftc.teamcode.opmodes.testing

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition
import dev.nextftc.hardware.positionable.SetPositions

@TeleOp(name = "Servo Test", group = "Config")
@Configurable
class ServoTest : NextFTCOpMode() {
    companion object {
        @JvmField var servoPos = 0.0;
    }
    val servo = ServoEx("turret_theta")

    override fun onStartButtonPressed() { }

    override fun onUpdate() {
        SetPosition(servo, servoPos)()
    }
}