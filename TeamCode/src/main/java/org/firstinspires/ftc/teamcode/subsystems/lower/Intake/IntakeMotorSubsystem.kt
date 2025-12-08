package org.firstinspires.ftc.teamcode.subsystems.lower.Intake

import com.acmerobotics.dashboard.config.Config
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower
import kotlinx.coroutines.Runnable
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeSubsystem
import java.util.function.Supplier

object IntakeMotorSubsystem : Subsystem {

  private val motor = MotorEx("intake");


  val OUT = -1.0;
  val IN = 1.0;

  class Power(power: Double) : InstantCommand({ motor.power = power })

  val intake = SetPower(motor, IN);
  val reverse = SetPower(motor, OUT);
  val stop = SetPower(motor, 0.0);

  override fun initialize() {
    motor.zero()
    motor.power = 0.0
  }

  class DriverCommandDefaultOn(  // could be bad for power draw?
    private val outPower: Supplier<Double>,
  ) : Command() {
    override val isDone = false;

    init {
      setInterruptible(true);
      setRequirements(IntakeSubsystem);
    }

    override fun update() {
      motor.power = 1.0 - 2.0 * outPower.get();
    }
  }

  class DriverCommand(
    private val inPower: Supplier<Double>,
    private val outPower: Supplier<Double>,
    private val callbackOnIntake: kotlinx.coroutines.Runnable = Runnable { },
    private val callbackOnReverse: Runnable = Runnable { },
    private val callbackOnRest: Runnable = Runnable { }
  ) : Command() {
    override val isDone = false;

    init {
      setName("Intake Drive")
      setRequirements(IntakeSubsystem);
    }

    var lastPower = 0.0;
    override fun update() {
      val power = inPower.get() - outPower.get();
      motor.power = power;
      if (power > 0.0 && lastPower <= 0.0) {
        callbackOnIntake.run();
      } else if (power < 0.0 && lastPower >= 0.0) {
        callbackOnReverse.run();
      } else if (power == 0.0 && lastPower != 0.0) {
        callbackOnRest.run();
      }
      lastPower = power;
    }
  }
}
