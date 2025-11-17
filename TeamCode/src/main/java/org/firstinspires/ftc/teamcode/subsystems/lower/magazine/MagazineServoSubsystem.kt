package org.firstinspires.ftc.teamcode.subsystems.lower.magazine

import com.acmerobotics.dashboard.config.Config
import dev.nextftc.core.commands.Command
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.CRServoEx
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeSubsystem
import java.util.function.Supplier

@Config
object MagazineServoSubsystem : Subsystem {
    @JvmField var FORWARD = 1.0;
    @JvmField var REVERSE = -1.0;

    private val servo = CRServoEx("magazine");

    val forward = SetPower(servo, FORWARD);
    val reverse = SetPower(servo, REVERSE);
    val stop = SetPower(servo, 0.0);

    class DriverCommandDefaultOn(  // could be bad for power draw?
        private val reversePower: Supplier<Double>,
    ) : Command() {
        override val isDone = false;

        init {
            setInterruptible(true);
            setRequirements(MagazineServoSubsystem);
        }

        override fun update() {
            servo.power = 1.0 - 2.0 * reversePower.get();
        }
    }

    class DriverCommand(
        private val forwardPower: Supplier<Double>,
        private val reversePower: Supplier<Double>,
    ) : Command() {
        override val isDone = false;

        init {
            setInterruptible(true);
            setRequirements(MagazineServoSubsystem);
        }

        override fun update() {
            servo.power = forwardPower.get() - reversePower.get();
        }
    }
}
