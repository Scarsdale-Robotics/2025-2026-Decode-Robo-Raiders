package org.firstinspires.ftc.teamcode.opmodes.testing

import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.deg
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.opmodes.testing.AutoAimTest.Companion.shootAngleDegrees
import org.firstinspires.ftc.teamcode.opmodes.testing.AutoAimTest.Companion.speed
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem

@TeleOp(name = "Tele Op In Prog")
class TeleOpInProg : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(
                IntakeSubsystem
            ),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onStartButtonPressed() {
        val intakeDrive = IntakeSubsystem.DriverCommandDefaultOn(
            Gamepads.gamepad1.leftTrigger,
        );
        intakeDrive.schedule();
    }

    override fun onUpdate() {
        PanelsTelemetry.telemetry.addData("CM", CommandManager.snapshot.toString());
        PanelsTelemetry.telemetry.update();
    }
}