package org.firstinspires.ftc.teamcode.opmodes.testing.baseSubsystems

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeServoSubsystem

@TeleOp(name = "Intake Servo Test", group = "Base Subsystem Tests")
class IntakeServoTest : NextFTCOpMode() {
    companion object {
        @JvmField var servoPos = 0.0;
    }

    val servo = ServoEx("intake_servo")

    override fun onStartButtonPressed() { }

    override fun onUpdate() {
        SetPosition(servo, servoPos)()
    }
}