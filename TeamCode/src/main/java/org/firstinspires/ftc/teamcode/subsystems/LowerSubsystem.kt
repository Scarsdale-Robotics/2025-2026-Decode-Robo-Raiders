package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.subsystems.SubsystemGroup
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagazineSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.MagazineServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.PusherServoSubsystem
import kotlin.time.Duration.Companion.milliseconds

object LowerSubsystem : SubsystemGroup(
    IntakeSubsystem,
    MagazineSubsystem,
) {
    @JvmField var FIX_JAM_DELAY_MS = 1000;

    @JvmField var EXTRA_DELAY_BETWEEN_LAUNCHES_MS = 500;

    @JvmField var fixJam = SequentialGroup(
//        ParallelGroup(
//            DriverCommandDefaultOn { 1.0 },  // 1.0 --> full reverse
//            MagazineServoSubsystem.reverse,
//        ),
//        MagblockServoSubsystem.open,
//        Delay(FIX_JAM_DELAY_MS.milliseconds),
//        MagblockServoSubsystem.close,
//        MagazineServoSubsystem.forward,  // doesn't reset intake b/c we don't know what intake was set at
    ).setInterruptible(false);

    val launch = SequentialGroup(
        PusherServoSubsystem.push,
        ParallelGroup(
            MagazineServoSubsystem.forward,
            SequentialGroup(
                Delay(100.milliseconds),
                IntakeSubsystem.forward,
            )
        ),
        Delay(EXTRA_DELAY_BETWEEN_LAUNCHES_MS.milliseconds),
    ).setInterruptible(false).requires(
        PusherServoSubsystem,
        MagazineServoSubsystem,
        IntakeSubsystem
    );
}
