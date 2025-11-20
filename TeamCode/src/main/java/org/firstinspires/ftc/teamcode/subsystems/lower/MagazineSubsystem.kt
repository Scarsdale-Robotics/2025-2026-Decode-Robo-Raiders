package org.firstinspires.ftc.teamcode.subsystems.lower

import com.acmerobotics.dashboard.config.Config
import dev.nextftc.core.subsystems.SubsystemGroup
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.MagazineServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.MagblockServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.PusherServoSubsystem

@Config
object MagazineSubsystem : SubsystemGroup(
    MagblockServoSubsystem,
    MagazineServoSubsystem,
    PusherServoSubsystem
)
