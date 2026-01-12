package org.firstinspires.ftc.teamcode.opmodes.testing.baseSubsystems

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.LowerMotorSubsystem

@Configurable
@TeleOp(name = "Lower Motor Test", group = "Base Subsystem Tests")
class LowerMotorTest : NextFTCOpMode() {
    companion object {
        @JvmField var power = 0.0;
    }

    init {
        addComponents(
            SubsystemComponent(LowerMotorSubsystem),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onStartButtonPressed() {
        LowerMotorSubsystem.On(power)();
//        val lowerMotorDrive = LowerMotorSubsystem.DriverCommandDefaultOn(
//            Gamepads.gamepad2.leftTrigger
//        );
//        lowerMotorDrive();
    }
}