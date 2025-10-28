package org.firstinspires.ftc.teamcode.opmodes.testing

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.deg
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem

@Config
@TeleOp(name = "Auto Aim Test")
class AutoAimTest : NextFTCOpMode() {
    @JvmField var shootAngleDegrees = 45.0;


    init {
        addComponents(
            SubsystemComponent(
                ShooterSubsystem,
                TurretThetaSubsystem
            )
        );
    }

    override fun onStartButtonPressed() {
        telemetry.addData("111", "111");
//        ShooterSubsystem.Run().schedule();
        telemetry.addData("CM", CommandManager.snapshot.toString());
        telemetry.update();
    }

    override fun onUpdate() {
        telemetry.addData("AAA", "AAA");
        TurretThetaSubsystem.targetTheta = shootAngleDegrees.deg;
        telemetry.addData("CM", CommandManager.snapshot.toString());
        telemetry.update();
    }
}
