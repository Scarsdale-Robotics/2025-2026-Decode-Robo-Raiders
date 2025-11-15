package org.firstinspires.ftc.teamcode.opmodes.testing

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.rad
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometrySubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem
import kotlin.math.atan2

@TeleOp(name = "Tele Op In Prog")
@Configurable
class TeleOpInProg : NextFTCOpMode() {
    companion object {
        var overaimSecs = 0.0;
    }
    private var odom: OdometrySubsystem? = null

    init {
        addComponents(
            SubsystemComponent(
                IntakeSubsystem,
                TurretPhiSubsystem,
            ),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onInit() {
        odom = OdometrySubsystem(0.0, 0.0, 0.0, hardwareMap)
    }

    override fun onStartButtonPressed() {
        // Initialize the device
        odom!!.setPinpoint(24.0*3.0, 24.0*3.0, Math.PI / 2.0);

        val intakeDrive = IntakeSubsystem.DriverCommandDefaultOn(
            Gamepads.gamepad1.leftTrigger,
        );
        intakeDrive.schedule();

        val goalX = 12.0;
        val goalY = 144.0-12.0;

        val autoAimPhi = TurretPhiSubsystem.AutoAim(
            { goalX - odom!!.rOx1 + odom!!.vx * overaimSecs },
            { goalY - odom!!.rOy1 + odom!!.vy * overaimSecs },
            { odom!!.rOh.rad + odom!!.omega.rad * overaimSecs }
        );
        autoAimPhi.schedule();
    }

    override fun onUpdate() {
        odom!!.updateOdom()

        val goalX = 12.0;
        val goalY = 144.0-12.0;
        PanelsTelemetry.telemetry.addData("ang degs", (atan2(goalY - odom!!.rOy1, goalX - odom!!.rOx1).rad - odom!!.rOh.rad).inDeg)

        PanelsTelemetry.telemetry.addData("x (inch)", odom!!.rOx1)
        PanelsTelemetry.telemetry.addData("y (inch)", odom!!.rOy1)
        PanelsTelemetry.telemetry.addData("h (radians)", odom!!.rOh)
        PanelsTelemetry.telemetry.addData("distance", odom!!.distance)

        PanelsTelemetry.telemetry.addData("CM", CommandManager.snapshot.toString());
        PanelsTelemetry.telemetry.update();
    }
}