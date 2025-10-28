package org.firstinspires.ftc.teamcode.subsystems.lower

import com.acmerobotics.dashboard.config.Config
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.subsystems.SubsystemGroup
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.MagazineMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.MagblockServoSubsystem
import kotlin.time.Duration.Companion.milliseconds

@Config
object MagazineSubsystem : SubsystemGroup(
    MagblockServoSubsystem,
    MagazineMotorSubsystem
)
