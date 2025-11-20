package org.firstinspires.ftc.teamcode.opmodes.testing

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.hardware.impl.CRServoEx
import dev.nextftc.hardware.powerable.SetPower

@TeleOp(name = "Mag Servo Test", group = "Config")
@Configurable
class MagServoTest : NextFTCOpMode() {
    private val servoL = CRServoEx("mwl");
    private val servoR = CRServoEx("mwr");

    companion object {
        var powerL = 0.0;
        var powerR = 0.0;
    }

    override fun onUpdate() {
        if (powerL != 0.0)
            SetPower(servoL, powerL)();
        if (powerR != 0.0)
            SetPower(servoR, -powerR)();

        PanelsTelemetry.telemetry.addData("CM", CommandManager.snapshot);
        PanelsTelemetry.telemetry.update();
    }
}