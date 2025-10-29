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
//@Config
class AutoAimTest : NextFTCOpMode() {
//    @JvmField var shootAngleDegrees = 45.0;

    init {
        addComponents(
            SubsystemComponent(
//                ShooterSubsystem,
                TurretThetaSubsystem
            ),
            BulkReadComponent,
            BindingsComponent
        );
    }

    override fun onStartButtonPressed() {
        telemetry.addData("111", "111");
//        ShooterSubsystem.Run();
        telemetry.addData("CM", CommandManager.snapshot.toString());
        telemetry.update();
    }

    override fun onUpdate() {
//        CommandManager.run()
//        SequentialGroup(open).schedule()

        telemetry.addData("AAA", "AAA");
        telemetry.addData("sdas", ServoEx("turret_theta").servo.portNumber);
        ServoEx("turret_theta").servo.position = 0.5
//        TurretThetaSubsystem.targetTheta = shootAngleDegrees.deg;
        telemetry.addData("CM", CommandManager.snapshot.toString());
        telemetry.update();
    }
}
