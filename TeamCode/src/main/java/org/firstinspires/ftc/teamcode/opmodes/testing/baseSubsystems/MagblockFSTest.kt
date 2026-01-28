package org.firstinspires.ftc.teamcode.opmodes.testing.baseSubsystems

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition
import dev.nextftc.hardware.powerable.SetPower

@TeleOp(name = "Magblock FS Test", group = "Base Subsystem Tests")
@Configurable
class MagblockFSTest : NextFTCOpMode() {
    companion object {
        @JvmField var servoPos = 0.02;
    }


    val servo = ServoEx("magblock")

    override fun onStartButtonPressed() { }

    override fun onUpdate() {
        SetPosition(servo, servoPos)()
    }
}