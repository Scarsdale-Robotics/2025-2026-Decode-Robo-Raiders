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
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition
import org.firstinspires.ftc.teamcode.opmodes.testing.TeleOpInProg.Companion.goalX
import org.firstinspires.ftc.teamcode.opmodes.testing.TeleOpInProg.Companion.goalY
import org.firstinspires.ftc.teamcode.opmodes.testing.TeleOpInProg.Companion.overaimSecs
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometrySubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem.open
import kotlin.math.hypot

@TeleOp(name = "Auto Aim Test 2")
@Configurable
class AutoAimTest2 : NextFTCOpMode() {
    init {
        addComponents(
            SubsystemComponent(
                ShooterSubsystem,
            ),
            BulkReadComponent,
            BindingsComponent
        );
    }

    private var odom: OdometrySubsystem? = null;

//    val x: Double
//        get() {
//            if (odom != null) {
//                return odom!!.rOx1;
//            }
//            return 0.0;
//        }
//    val y: Double
//        get() {
//            if (odom != null) {
//                return odom!!.rOy1;
//            }
//            return 0.0;
//        }
//    val h: Angle
//        get() {
//            if (odom != null) {
//                return odom!!.rOh.rad;
//            }
//            return 0.0.rad;
//        }

    override fun onInit() {
//        odom = LocalizationSubsystem(0.0, 0.0, 0.0, isBlue, hardwareMap)
        odom = OdometrySubsystem(0.0, 0.0, 0.0, hardwareMap)
    }

    override fun onStartButtonPressed() {
//        telemetry.addData("111", "111");
//        ShooterSubsystem.on();
        odom!!.setPinpoint(72.0, 72.0, -Math.PI / 2.0);

//        val autoAimPhi = TurretPhiSubsystem.AutoAim(
//            { goalX - odom!!.rOx1 },
//            { goalY - odom!!.rOy1 },
//            { odom!!.rOh.rad }
//        );
//        autoAimPhi.schedule();
        val shooterAutoAim = ShooterSubsystem.AutoAim(
            { hypot(goalX - odom!!.rOx1, goalY - odom!!.rOy1) },
            { (831 + 67 + 3.52*it - 0.00429*it*it).coerceIn(0.0, 1500.0) }
        )
        shooterAutoAim.schedule();
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
//        PanelsTelemetry.telemetry.addData("CM", CommandManager.snapshot.toString());
//        PanelsTelemetry.telemetry.update();
//        telemetry.addData("CM", CommandManager.snapshot.toString());
//        telemetry.update();
    }

    override fun onStop() {
        CommandManager.cancelAll()
    }
}
