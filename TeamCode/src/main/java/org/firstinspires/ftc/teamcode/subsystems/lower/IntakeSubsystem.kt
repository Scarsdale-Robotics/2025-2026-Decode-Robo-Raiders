package org.firstinspires.ftc.teamcode.subsystems.lower

import com.acmerobotics.dashboard.config.Config
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.core.subsystems.SubsystemGroup
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower
import kotlinx.coroutines.Runnable
import org.firstinspires.ftc.teamcode.subsystems.lower.Intake.IntakeMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.Intake.IntakeServoSubsystem
import java.util.function.Supplier

@Config
object IntakeSubsystem :SubsystemGroup(
  IntakeServoSubsystem,
  IntakeMotorSubsystem
)
