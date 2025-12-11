package org.firstinspires.ftc.teamcode.subsystems.lower

import com.acmerobotics.dashboard.config.Config
import dev.nextftc.core.subsystems.SubsystemGroup
import org.firstinspires.ftc.teamcode.subsystems.lower.intake.IntakeMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.intake.IntakeServoSubsystem

@Config
object IntakeSubsystem :SubsystemGroup(
  IntakeServoSubsystem,
  IntakeMotorSubsystem
)
