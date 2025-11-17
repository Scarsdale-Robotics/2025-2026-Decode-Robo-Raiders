package org.firstinspires.ftc.teamcode.subsystems.lower

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

    class Power(power: Double) : InstantCommand({ motor.power = power })

    val forward = SetPower(motor, FORWARD);
    val reverse = SetPower(motor, REVERSE);
    val stop = SetPower(motor, 0.0);

    override fun initialize() {
        motor.zero()
        motor.power = 0.0
    }

    class DriverCommandDefaultOn(  // could be bad for power draw?
        private val reversePower: Supplier<Double>,
    ) : Command() {
        override val isDone = false;

        init {
            setInterruptible(true);
            setRequirements(IntakeSubsystem);
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

        init {
            setInterruptible(true);
            setRequirements(IntakeSubsystem);
        }

        override fun update() {
            motor.power = forwardPower.get() - reversePower.get();
        }
    }
}