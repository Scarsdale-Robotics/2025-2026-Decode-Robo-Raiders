package org.firstinspires.ftc.teamcode.subsystems.lower.magazine

import com.acmerobotics.dashboard.config.Config
import dev.nextftc.core.commands.Command
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.CRServoEx
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower
import java.util.function.Supplier

@Config
object MagazineServoSubsystem : Subsystem {
    @JvmField var FORWARD = 1.0;
    @JvmField var REVERSE = -1.0;

    private val servo = CRServoEx("magazine");

    @JvmStatic val forward = SetPower(servo, FORWARD);
    @JvmStatic val reverse = SetPower(servo, REVERSE);


    class DriverCommandDefaultOn(  // could be bad for power draw?
        private val reversePower: Supplier<Double>,
    ) : Command() {
        override val isDone = false;

        override fun start() {
            servo.power = 0.0;
        }

        override fun update() {
            servo.power = 1.0 - 2.0 * reversePower.get();
        }
    }
}
