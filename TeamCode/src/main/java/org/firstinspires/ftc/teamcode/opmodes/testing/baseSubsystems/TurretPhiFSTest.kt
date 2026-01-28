package org.firstinspires.ftc.teamcode.opmodes.testing.baseSubsystems

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.deg
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem

@TeleOp(name = "Turret Phi FS Test", group = "Base Subsystem Tests")
@Configurable
class TurretPhiFSTest : NextFTCOpMode() {

    companion object {
        var degrees = 0.0;
    }

    override fun onInit() {
        addComponents(
            SubsystemComponent(TurretPhiSubsystem),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onUpdate() {
        CommandManager.cancelAll()
        TurretPhiSubsystem.SetTargetPhi(degrees.deg)();
        PanelsTelemetry.telemetry.update()
    }

}