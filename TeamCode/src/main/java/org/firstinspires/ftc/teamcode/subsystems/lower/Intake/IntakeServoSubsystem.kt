package org.firstinspires.ftc.teamcode.subsystems.lower.Intake

import com.acmerobotics.dashboard.config.Config
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.CRServoEx
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition
import dev.nextftc.hardware.powerable.SetPower
import java.util.function.Supplier

@Config
object IntakeServoSubsystem : Subsystem {
  @JvmField var FORWARD = 1.0;
  @JvmField var REVERSE = -1.0;

  private val servo = ServoEx("IntakeServo");

  val up = SetPosition(servo, -FORWARD)
  val down = SetPosition(servo, -REVERSE)

}
