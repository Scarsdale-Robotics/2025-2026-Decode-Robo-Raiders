package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.core.subsystems.SubsystemGroup
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.LowerMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagblockServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.TurretSubsystem

object LowerSubsystem : SubsystemGroup(
    IntakeServoSubsystem,
    LowerMotorSubsystem,
    MagblockServoSubsystem
)