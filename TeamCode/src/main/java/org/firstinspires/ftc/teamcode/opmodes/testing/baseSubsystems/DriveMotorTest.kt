package org.firstinspires.ftc.teamcode.opmodes.testing.baseSubsystems

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeMotorSubsystem

@Configurable
@TeleOp(name = "Drive Motor Test", group = "Base Subsystem Tests")
class DriveMotorTest : NextFTCOpMode() {
    companion object {
        @JvmField var powerlfw = 0.0;
        @JvmField var powerlbw = 0.0;
        @JvmField var powerrfw = 0.0;
        @JvmField var powerrbw = 0.0;
    }
    private val lfw = MotorEx("lfw").reversed();
    private val lbw = MotorEx("lbw").reversed();
    private val rfw = MotorEx("rfw");
    private val rbw = MotorEx("rbw");

    init {
        addComponents(
            SubsystemComponent(IntakeMotorSubsystem),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onUpdate() {
        CommandManager.cancelAll()
        SetPower(lfw, powerlfw)()
        SetPower(rbw, powerrbw)()
        SetPower(rfw, powerrfw)()
        SetPower(lbw, powerlbw)()
//        val lowerMotorDrive = LowerMotorSubsystem.DriverCommandDefaultOn(
//            Gamepads.gamepad2.leftTrigger
//        );
//        lowerMotorDrive();
    }
}