package org.firstinspires.ftc.teamcode.opmodes.testing

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem

@TeleOp(name = "Turret Phi Test", group = "Config")
@Configurable
class TurretPhiTest : NextFTCOpMode() {
    companion object {
        var angle = 45.0;
    }
    init {
        addComponents(
            SubsystemComponent(
                TurretPhiSubsystem
            ),
            BindingsComponent,
            BulkReadComponent,
        )
    }

    override fun onStartButtonPressed() {

    }

    override fun onUpdate() {
        CommandManager.cancelAll()
        TurretPhiSubsystem.SetTargetPhi(angle.deg)()
        PanelsTelemetry.telemetry.update()
    }
}