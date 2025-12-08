package org.firstinspires.ftc.teamcode.subsystems.lower.magazine

import dev.nextftc.control.ControlSystem
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.nextftc.core.commands.Command
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeSubsystem
import java.util.function.Supplier

object MagazineMotorSubsystem : Subsystem {
    private val motor = MotorEx("mw")

    var slowSpeed = 300.0  // magblock safe
    var fastSpeed = 900.0  // magblock unsafe

    var speedFactor = 0.0;

    @JvmField var ffCoefficients = BasicFeedforwardParameters(0.0, 0.0, 0.75);
    @JvmField var pidCoefficients = PIDCoefficients(0.016, 0.0, 0.0)

    private val controller: ControlSystem;

    init {
        controller = controlSystem {
            basicFF(ffCoefficients)
            velPid(pidCoefficients)
        }
    }

    class DriverCommandDefaultOn(
        private val outPower: Supplier<Double>,
    ) : Command() {
        override val isDone = false;

        init {
            setInterruptible(true);
            setRequirements(MagazineMotorSubsystem);
        }

        override fun update() {
            speedFactor = 1.0 - 2.0 * outPower.get();
        }
    }

    override fun periodic() {
        val baseSpeed = if (MagblockServoSubsystem.isOpen) fastSpeed else slowSpeed;
        SetPower(motor, baseSpeed * speedFactor)()
    }
}