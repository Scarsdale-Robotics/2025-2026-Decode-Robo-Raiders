package org.firstinspires.ftc.teamcode.subsystems.lower.magazine

import dev.nextftc.control.ControlSystem
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.nextftc.core.commands.Command
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.controllable.RunToState
import dev.nextftc.hardware.controllable.RunToVelocity
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
import java.util.function.Supplier

object MagazineMotorSubsystem : Subsystem {
    private val motor = MotorEx("mw")

    var SLOW_SPEED = 300.0  // magblock safe
    var FAST_SPEED = 900.0  // magblock unsafe

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

    class On(speed: Double) : RunToState(controller, KineticState(velocity=speed))
    var fast = RunToVelocity(controller, FAST_SPEED).requires(this).named("MagFast").setInterruptible(true);
    var slow = RunToVelocity(controller, SLOW_SPEED).requires(this).named("MagSlow").setInterruptible(true);
    @JvmField var off = RunToVelocity(controller, 0.0).requires(this).named("MagOff").setInterruptible(true);

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
        val power = controller.calculate(
            KineticState(motor.currentPosition)
        ).coerceIn(0.0, 1.0)
        SetPower(motor, power * speedFactor)()
    }
}