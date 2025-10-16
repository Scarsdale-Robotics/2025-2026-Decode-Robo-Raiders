package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.subsystems.SubsystemGroup
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeSubsystem.DriverCommandDefaultOn
import org.firstinspires.ftc.teamcode.subsystems.lower.MagazineSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.MagazineMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.MagblockServoSubsystem
import kotlin.time.Duration.Companion.milliseconds

object LowerSubsystem : SubsystemGroup(
    IntakeSubsystem,
    MagazineSubsystem,
) {
    @JvmField var FIX_JAM_DELAY_MS = 1000;

    @JvmField var fixJam = SequentialGroup(
        ParallelGroup(
            DriverCommandDefaultOn { 1.0 },  // 1.0 --> full reverse
            MagazineMotorSubsystem.Reverse(),
        ),
        MagblockServoSubsystem.Open(),
        Delay(FIX_JAM_DELAY_MS.milliseconds),
        MagblockServoSubsystem.Close(),
        MagazineMotorSubsystem.Forward(),  // doesn't reset intake b/c we don't know what intake was set at
    ).setInterruptible(false);
}
