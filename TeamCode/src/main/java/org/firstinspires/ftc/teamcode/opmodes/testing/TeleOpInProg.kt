package org.firstinspires.ftc.teamcode.opmodes.testing

import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometrySubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeSubsystem

@TeleOp(name = "Tele Op In Prog")
class TeleOpInProg : NextFTCOpMode() {
    private var odom: OdometrySubsystem? = null

    init {
        addComponents(
            SubsystemComponent(
                IntakeSubsystem
            ),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onInit() {
        odom = OdometrySubsystem(24.0*3.0, 24.0*3.0, Math.PI / 2.0, hardwareMap)
    }

    override fun onStartButtonPressed() {
        odom!!.setPinpoint(24.0*3.0, 24.0*3.0, Math.PI / 2.0);

        val intakeDrive = IntakeSubsystem.DriverCommandDefaultOn(
            Gamepads.gamepad1.leftTrigger,
        );
        intakeDrive.schedule();
    }

    override fun onUpdate() {
        odom!!.updateOdom()

        PanelsTelemetry.telemetry.addData("x (inch)", odom!!.rOx1)
        PanelsTelemetry.telemetry.addData("y (inch)", odom!!.rOy1)
        PanelsTelemetry.telemetry.addData("h (radians)", odom!!.rOh)
        PanelsTelemetry.telemetry.addData("distance", odom!!.distance)

        PanelsTelemetry.telemetry.addData("CM", CommandManager.snapshot.toString());
        PanelsTelemetry.telemetry.update();
    }
}