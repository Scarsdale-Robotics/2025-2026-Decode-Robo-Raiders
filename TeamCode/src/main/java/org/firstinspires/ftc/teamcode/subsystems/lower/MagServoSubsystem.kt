package org.firstinspires.ftc.teamcode.subsystems.lower

import dev.nextftc.bindings.Button
import dev.nextftc.core.commands.Command
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.CRServoEx
import dev.nextftc.hardware.powerable.SetPower

object MagServoSubsystem: Subsystem {
    private val servo = CRServoEx("MagServo");

    val run = SetPower(servo, -1.0)
    val stop = SetPower(servo, 0.0)

    class DriverCommandDefaultOn(
        private val reverse: Button
    ) : Command() {
        override val isDone = false;

        init {
            setRequirements(MagServoSubsystem)
        }

        override fun update() {
            if (reverse.get()) servo.power = 1.0
            else servo.power = -1.0
        }
    }
}