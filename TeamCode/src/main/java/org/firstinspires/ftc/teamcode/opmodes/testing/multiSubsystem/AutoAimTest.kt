//package org.firstinspires.ftc.teamcode.opmodes.testing.multiSubsystem
//
//import com.bylazar.configurables.annotations.Configurable
//import com.bylazar.telemetry.PanelsTelemetry
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp
//import dev.nextftc.core.commands.CommandManager
//import dev.nextftc.core.components.BindingsComponent
//import dev.nextftc.core.components.SubsystemComponent
//import dev.nextftc.core.units.Angle
//import dev.nextftc.core.units.deg
//import dev.nextftc.core.units.rad
//import dev.nextftc.ftc.NextFTCOpMode
//import dev.nextftc.ftc.components.BulkReadComponent
//import dev.nextftc.hardware.impl.ServoEx
//import dev.nextftc.hardware.positionable.SetPosition
//import org.firstinspires.ftc.teamcode.opmodes.testing.baseSubsystems.TurretThetaTest
//import org.firstinspires.ftc.teamcode.subsystems.localization.OdometrySubsystem
//import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
//import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem
//import kotlin.math.PI
//
//
//@TeleOp(name = "Auto Aim Test", group = "Multi Subsystem Tests")
//@Configurable
//class AutoAimTest : NextFTCOpMode() {
//    companion object {
//        //@JvmField var servoPos = 0.0;
//        @JvmField var shootAngleDegrees = 60;
//        @JvmField var speed = 1423.0;
//    }
//
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
//
//    private var odom: OdometrySubsystem? = null;
//
//    override fun onInit() {
//        addComponents(
//            SubsystemComponent(
//                ShooterSubsystem,
//                TurretThetaSubsystem
//            ),
//            BulkReadComponent,
//            BindingsComponent
//        );
//        odom = OdometrySubsystem(72.0, 72.0, PI / 2, hardwareMap)
//    }
//
//    val servo = ServoEx("turret_theta")
//
//    override fun onStartButtonPressed() {
//        odom!!.setPinpoint(72.0, 72.0, PI / 2)
////        telemetry.addData("111", "111");
////        ShooterSubsystem.on();
//    }
//
//    override fun onUpdate() {
////        val dashboard = FtcDashboard.getInstance()
////        val dashboardTelemetry = dashboard.telemetry
////        CommandManager.run()
//        //
////        SequentialGroup(open).schedule()
//        CommandManager.cancelAll()
//        ShooterSubsystem.On(speed).setInterruptible(true)
//        //(SetPosition(servo, servoPos)()
//        ///0.2-0.93///
//        TurretThetaSubsystem.SetTargetTheta(shootAngleDegrees.deg)()
//
////        telemetry.addData("AAA", "AAA");
////        telemetry.addData("sdas", ServoEx("turret_theta").servo.portNumber);
////        ServoEx("turret_theta").servo.position = 0.5
////        TurretThetaSubsystem.targetTheta = shootAngleDegrees.deg;
//        PanelsTelemetry.telemetry.addData("CM", CommandManager.snapshot.toString());
//        PanelsTelemetry.telemetry.update();
//        telemetry.addData("CM", CommandManager.snapshot.toString());
//        telemetry.update();
//    }
//
//    override fun onStop() {
//        CommandManager.cancelAll()
//    }
//}