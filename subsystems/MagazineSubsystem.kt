package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.commands.utility.LambdaCommand
import dev.nextftc.core.subsystems.SubsystemGroup
import org.firstinspires.ftc.teamcode.subsystems.magazine.MagazineMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.magazine.MagblockServoSubsystem
import kotlin.time.Duration.Companion.milliseconds

@Config
object MagazineSubsystem : SubsystemGroup(
    MagblockServoSubsystem,
    MagazineMotorSubsystem
) {
    @JvmField var FIX_JAM_DELAY_MS = 1000;

    val fixJam = SequentialGroup(
        MagazineMotorSubsystem.reverse,
        MagblockServoSubsystem.open,
        Delay(FIX_JAM_DELAY_MS.milliseconds),
        MagblockServoSubsystem.close,
        MagazineMotorSubsystem.forward,
    );
}