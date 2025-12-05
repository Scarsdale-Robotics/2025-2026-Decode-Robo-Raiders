package org.firstinspires.ftc.teamcode.opmodes.testing

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.opmodes.testing.TeleOpInProg.Companion.distanceGoalX
import org.firstinspires.ftc.teamcode.opmodes.testing.TeleOpInProg.Companion.distanceGoalY
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometrySubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
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
            { hypot(distanceGoalX - odom!!.rOx1, distanceGoalY - odom!!.rOy1) },
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
