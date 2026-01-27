package org.firstinspires.ftc.teamcode.opmodes.testing.baseSubsystems

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagMotorSubsystem

@Configurable
@TeleOp(name = "Intake Motor Test", group = "Base Subsystem Tests")
class IntakeMotorTest : NextFTCOpMode() {
    companion object {
        @JvmField var power = 0.0;
    }

    init {
        addComponents(
            SubsystemComponent(IntakeMotorSubsystem),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onUpdate() {
        CommandManager.cancelAll()
        IntakeMotorSubsystem.On(power)();
//        val lowerMotorDrive = LowerMotorSubsystem.DriverCommandDefaultOn(
//            Gamepads.gamepad2.leftTrigger
//        );
//        lowerMotorDrive();
    }
}