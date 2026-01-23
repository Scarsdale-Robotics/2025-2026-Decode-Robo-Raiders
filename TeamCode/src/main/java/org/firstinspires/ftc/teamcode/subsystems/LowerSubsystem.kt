package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.core.subsystems.SubsystemGroup
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagblockServoSubsystem

object LowerSubsystem : SubsystemGroup(
    IntakeServoSubsystem,
    MagMotorSubsystem,
    MagblockServoSubsystem,
    MagServoSubsystem
)