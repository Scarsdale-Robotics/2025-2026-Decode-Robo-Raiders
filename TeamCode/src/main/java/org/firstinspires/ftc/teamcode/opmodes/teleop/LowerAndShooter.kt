package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.subsystems.LowerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.LowerMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem

@Configurable
@TeleOp(name = "Lower and Shooter")
class LowerAndShooter(): NextFTCOpMode() {

    companion object {
        @JvmField var speed1 = 0.0;
    }

    override fun onInit() {
        addComponents(
            SubsystemComponent(
                LowerSubsystem,
                OuttakeSubsystem
            ),
            BindingsComponent,
            BulkReadComponent
        )

        ShooterSubsystem.off()
        LowerMotorSubsystem.off()
    }

    override fun onStartButtonPressed() {

        val lowerMotorDrive = LowerMotorSubsystem.DriverCommandDefaultOn(
            Gamepads.gamepad1.leftTrigger
        );
        lowerMotorDrive();

        val magDrive = MagServoSubsystem.DriverCommandDefaultOn(
            Gamepads.gamepad1.leftTrigger.greaterThan(0.0)
        )
        magDrive();

    }

    override fun onUpdate() {
        ShooterSubsystem.On(speed1)();
        PanelsTelemetry.telemetry.update()
    }

}
