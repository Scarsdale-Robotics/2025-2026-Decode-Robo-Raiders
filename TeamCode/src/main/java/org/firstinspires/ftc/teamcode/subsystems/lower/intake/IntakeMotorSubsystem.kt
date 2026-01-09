package org.firstinspires.ftc.teamcode.subsystems.lower.intake

import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower
import kotlinx.coroutines.Runnable
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeSubsystem
import java.util.function.Supplier

object IntakeMotorSubsystem : Subsystem {

  private val motor = MotorEx("im");

//  class On(power: Double) : InstantCommand({ motor.power = power })
////  val off = InstantCommand({ motor.power = 0.0 })

  val intake = SetPower(motor, 1.0);
  val slow = SetPower(motor, 0.5);
  val reverse = SetPower(motor, -1.0);
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
