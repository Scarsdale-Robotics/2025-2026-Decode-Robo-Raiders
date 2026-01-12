package org.firstinspires.ftc.teamcode.opmodes.testing.baseSubsystems

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower

@TeleOp(name = "Turret Phi Test", group = "Base Subsystem Tests")
@Configurable
class TurretPhiTest : NextFTCOpMode() {
    val motor = MotorEx("turret_phi");

    companion object {
        var power = 0.5;
    }

//    override fun onStartButtonPressed() {
//        val forward = SetPower(motor, power);
//        val backward = SetPower(motor, -power);
//        val rest = SetPower(motor, 0.0);
//
//        Gamepads.gamepad1.rightBumper whenBecomesTrue forward whenBecomesFalse rest
//        Gamepads.gamepad1.leftBumper whenBecomesTrue backward whenBecomesFalse rest
//    }

    override fun onUpdate() {
        SetPower(motor, power)();

        PanelsTelemetry.telemetry.addData("CM", CommandManager.snapshot);
        PanelsTelemetry.telemetry.addData("enc", motor.currentPosition);
        PanelsTelemetry.telemetry.addData("time", runtime);
        PanelsTelemetry.telemetry.update();
    }
}