package org.firstinspires.ftc.teamcode.subsystems.intake

import dev.nextftc.core.commands.Command
import dev.nextftc.hardware.controllable.Controllable
import dev.nextftc.hardware.powerable.SetPower
import java.util.function.Supplier

class IntakeDriverCommand(
    private val intakeMotor: Controllable,
    private val forwardPower: Supplier<Double>,
    private val reversePower: Supplier<Double>
) : Command() {
    override val isDone = false;

    override fun update() {
        SetPower(intakeMotor, forwardPower.get() - reversePower.get());
    }
}