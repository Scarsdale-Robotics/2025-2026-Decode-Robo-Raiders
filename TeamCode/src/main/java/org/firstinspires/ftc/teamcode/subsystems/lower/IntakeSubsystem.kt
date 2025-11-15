package org.firstinspires.ftc.teamcode.subsystems.lower

import com.bylazar.configurables.annotations.Configurable
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower
import java.util.function.Supplier

object IntakeSubsystem : Subsystem {
    private val motor = MotorEx("intake");

    val REVERSE = -1.0;
    val FORWARD = 1.0;

    class SetPower(power: Double) : InstantCommand({ SetPower(motor, power); });

    val forward = SetPower(FORWARD);
    val reverse = SetPower(REVERSE);

    class DriverCommandDefaultOn(  // could be bad for power draw?
        private val reversePower: Supplier<Double>,
    ) : Command() {
        override val isDone = false;

        override fun start() {
            motor.power = 0.0;
        }

        override fun update() {
            motor.power = 1.0 - 2.0 * reversePower.get();
        }
    }

    class DriverCommand(
        private val forwardPower: Supplier<Double>,
        private val reversePower: Supplier<Double>,
    ) : Command() {
        override val isDone = false;

        override fun start() {
            motor.power = 0.0;
        }

        override fun update() {
            motor.power = forwardPower.get() - reversePower.get();
        }
    }
}