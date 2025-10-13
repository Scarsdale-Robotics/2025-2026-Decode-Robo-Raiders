package org.firstinspires.ftc.teamcode.subsystems.lower

import dev.nextftc.core.commands.Command
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower
import java.util.function.Supplier

object IntakeSubsystem : Subsystem {
    private val motor = MotorEx("intake");

    val REVERSE = -1.0;
    val FORWARD = 1.0;

    var power = 1.0
        set(value) {
            field = value.coerceIn(-1.0, 1.0);
        };

    override fun periodic() {
        SetPower(motor, power);
    }

    class DriverCommandDefaultOn(  // could be bad for power draw?
        private val reversePower: Supplier<Double>,
    ) : Command() {
        override val isDone = false;

        override fun update() {
            power = 1.0 - 2.0 * reversePower.get();
        }
    }

    class DriverCommand(
        private val forwardPower: Supplier<Double>,
        private val reversePower: Supplier<Double>,
    ) : Command() {
        override val isDone = false;

        override fun update() {
            power = forwardPower.get() - reversePower.get();
        }
    }
}