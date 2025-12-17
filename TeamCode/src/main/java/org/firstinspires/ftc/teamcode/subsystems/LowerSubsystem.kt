package org.firstinspires.ftc.teamcode.subsystems

import com.bylazar.configurables.annotations.Configurable
import dev.nextftc.core.commands.delays.Delay
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.subsystems.SubsystemGroup
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagazineSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.MagazineServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.MagblockServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.PusherServoSubsystem
import kotlin.time.Duration.Companion.milliseconds

@Configurable
object LowerSubsystem : SubsystemGroup(
    IntakeSubsystem,
    MagazineSubsystem,
) {
    @JvmField var FIX_JAM_DELAY_MS = 1000;

    @JvmField var EXTRA_DELAY_BETWEEN_LAUNCHES_MS = 500;
    @JvmField var LAUNCH_ALL_TIME_BEFORE_LAST_SHOOT_MS = 2000;

    @JvmField var BALL_UP_DELAY_MS = 2000;

    @JvmField var TOP_RISE_DELAY_MS = 2000;

//    var fixJam = SequentialGroup(
////        ParallelGroup(
////            DriverCommandDefaultOn { 1.0 },  // 1.0 --> full reverse
////            MagazineServoSubsystem.reverse,
////        ),
////        MagblockServoSubsystem.open,
////        Delay(FIX_JAM_DELAY_MS.milliseconds),
////        MagblockServoSubsystem.close,
////        MagazineServoSubsystem.forward,  // doesn't reset intake b/c we don't know what intake was set at
//    ).setInterruptible(false);

    val launchAll = SequentialGroup(
        MagblockServoSubsystem.open,
        Delay(LAUNCH_ALL_TIME_BEFORE_LAST_SHOOT_MS.milliseconds),
        PusherServoSubsystem.pushMult,
        MagblockServoSubsystem.close
    )

//    val launchOne = SequentialGroup(
//        MagazineServoSubsystem.forward,
//        Delay(TOP_RISE_DELAY_MS.milliseconds),
//        PusherServoSubsystem.pushMult,
//        MagblockServoSubsystem.open,
//        Delay(BALL_UP_DELAY_MS.milliseconds),
//        MagblockServoSubsystem.close,
//        IntakeSubsystem.intake,
////        Delay(EXTRA_DELAY_BETWEEN_LAUNCHES_MS.milliseconds),
////        MagazineServoSubsystem.stop,
////        IntakeSubsystem.stop,
//    ).setInterruptible(false).requires(
//        PusherServoSubsystem,
//        MagazineServoSubsystem,
//        IntakeSubsystem
//    );
}
