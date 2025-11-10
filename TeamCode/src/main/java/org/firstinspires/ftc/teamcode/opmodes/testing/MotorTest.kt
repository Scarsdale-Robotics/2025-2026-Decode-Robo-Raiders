package org.firstinspires.ftc.teamcode.opmodes.testing

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower

@TeleOp(name = "Motor Test", group = "Config")
@Configurable
class MotorTest : NextFTCOpMode() {
    val motor = MotorEx("intake");

    companion object {
        var power = 0.5;
    }

    override fun onUpdate() {
        SetPower(motor, power)();
        PanelsTelemetry.telemetry.addData("CM", CommandManager.snapshot);
        PanelsTelemetry.telemetry.update();
    }
}