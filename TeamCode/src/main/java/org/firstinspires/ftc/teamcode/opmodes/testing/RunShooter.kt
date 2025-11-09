package org.firstinspires.ftc.teamcode.opmodes.testing

import com.acmerobotics.dashboard.config.Config
import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem

@TeleOp(name = "Run Shooter")
@Configurable
class RunShooter : NextFTCOpMode() {
    init {
        addComponents(
            BulkReadComponent,
            BindingsComponent
        )
    }

    companion object {
        @JvmField var power = 0.5
    }

    val motor = MotorEx("shooter")

    override fun onStartButtonPressed() {
    }

    override fun onUpdate() {
        SetPower(motor, power)();
        telemetry.addData("CM", CommandManager.snapshot);
        telemetry.update();
    }
}