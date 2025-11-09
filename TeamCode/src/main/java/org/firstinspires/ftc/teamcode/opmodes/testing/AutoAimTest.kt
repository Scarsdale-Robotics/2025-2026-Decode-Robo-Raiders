package org.firstinspires.ftc.teamcode.opmodes.testing

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.deg
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem.open

@TeleOp(name = "Auto Aim Test")
@Configurable
class AutoAimTest : NextFTCOpMode() {
    companion object {
        @JvmField var shootAngleDegrees = 45.0;
    }

    init {
        addComponents(
            SubsystemComponent(
                ShooterSubsystem,
                TurretThetaSubsystem
            ),
            BulkReadComponent,
            BindingsComponent
        );
    }

    override fun onStartButtonPressed() {
//        telemetry.addData("111", "111");
//        ShooterSubsystem.on();
        Gamepads.gamepad1.circle whenBecomesTrue ShooterSubsystem.on
        Gamepads.gamepad1.square whenBecomesTrue ShooterSubsystem.off
        Gamepads.gamepad1.triangle whenBecomesTrue TurretThetaSubsystem.SetTargetTheta(shootAngleDegrees.deg)
    }

    override fun onUpdate() {
//        val dashboard = FtcDashboard.getInstance()
//        val dashboardTelemetry = dashboard.telemetry
//        CommandManager.run()
//        SequentialGroup(open).schedule()

//        telemetry.addData("AAA", "AAA");
//        telemetry.addData("sdas", ServoEx("turret_theta").servo.portNumber);
//        ServoEx("turret_theta").servo.position = 0.5
//        TurretThetaSubsystem.targetTheta = shootAngleDegrees.deg;
        PanelsTelemetry.telemetry.addData("CM", CommandManager.snapshot.toString());
        PanelsTelemetry.telemetry.update();
        telemetry.addData("CM", CommandManager.snapshot.toString());
        telemetry.update();
    }

    override fun onStop() {
        CommandManager.cancelAll()
    }
}
