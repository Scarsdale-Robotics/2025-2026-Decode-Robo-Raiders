package org.firstinspires.ftc.teamcode.opmodes.testing

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.deg
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem.open

@TeleOp(name = "Auto Aim Test")
@Config
class AutoAimTest : NextFTCOpMode() {
    companion object {
        @JvmField var angleDegs = 45.0;
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
//        ShooterSubsystem.Run()
    }

    override fun onUpdate() {
        TurretThetaSubsystem.targetTheta = angleDegs.deg;
        telemetry.addData("CM", CommandManager.snapshot.toString());
        telemetry.update();
    }
}
