package org.firstinspires.ftc.teamcode.opmodes.testing

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.subsystems.LowerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometrySubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagazineSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.MagazineServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.MagblockServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.PusherServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem
import kotlin.math.atan2
import kotlin.math.hypot

@TeleOp(name = "Tele Op In Prog")
@Configurable
class TeleOpInProg : NextFTCOpMode() {
    companion object {
        var overaimSecs = 0.0;
        var speed = 1100.0;
        var goalX = 12.0;
        var goalY = 144.0-12.0;
        var isBlue = true;
    }
//    private var odom: LocalizationSubsystem? = null
    private var odom: OdometrySubsystem? = null;

    init {
        addComponents(
            SubsystemComponent(
//                IntakeSubsystem,
                TurretPhiSubsystem,
//                MagazineServoSubsystem,
                ShooterSubsystem,
//                PusherServoSubsystem,
//                MagblockServoSubsystem,
//                MagazineSubsystem
                LowerSubsystem
            ),
            BulkReadComponent,
            BindingsComponent
        )
    }

    override fun onInit() {
//        odom = LocalizationSubsystem(0.0, 0.0, 0.0, isBlue, hardwareMap)
        odom = OdometrySubsystem(0.0, 0.0, 0.0, hardwareMap)
    }

    override fun onStartButtonPressed() {
        val startX = 24.0*3.0;
        val startY = 24.0*3.0;


        // Initialize the device
        odom!!.setPinpoint(startX, startY, Math.PI / 2.0);

        val intakeDrive = IntakeSubsystem.DriverCommand(
            Gamepads.gamepad1.rightTrigger,
            Gamepads.gamepad1.leftTrigger
        );
        intakeDrive.schedule();


        val magDrive = MagazineServoSubsystem.DriverCommand(
            Gamepads.gamepad1.rightTrigger,
            Gamepads.gamepad1.leftTrigger
        );
        magDrive.schedule();



//        val thetaAim = TurretThetaSubsystem.AutoAim(
//            {
//                hypot(
//                    goalX - odom!!.rOx1 + odom!!.vx * overaimSecs,
//                    goalY - odom!!.rOy1 + odom!!.vy * overaimSecs,
//                )
//            },
//            { (-0.123 * it + 63.0).coerceIn(45.0, 63.0).deg }
//        )
//        thetaAim(); // can tbis just be turned into calling it on the previous line?



//        val autoAimPhi = TurretPhiSubsystem.AutoAim(
//            { goalX - odom!!.rOx1 + odom!!.vx * overaimSecs },
//            { goalY - odom!!.rOy1 + odom!!.vy * overaimSecs },
//            { odom!!.rOh.rad + odom!!.omega.rad * overaimSecs }
//        );
//        autoAimPhi.schedule();



        Gamepads.gamepad1.square whenBecomesTrue LowerSubsystem.launch
    }

    var onCmd: ShooterSubsystem.On? = null;
    override fun onUpdate() {
        odom!!.updateOdom()

        if (onCmd != null) CommandManager.cancelCommand(onCmd!!)
        val cmd = ShooterSubsystem.On(speed);
        cmd();
        onCmd = cmd;

        PanelsTelemetry.telemetry.addData("ang degs", (atan2(goalY - odom!!.rOy1, goalX - odom!!.rOx1).rad - odom!!.rOh.rad).inDeg)

        PanelsTelemetry.telemetry.addData("x (inch)", odom!!.rOx1)
        PanelsTelemetry.telemetry.addData("y (inch)", odom!!.rOy1)
        PanelsTelemetry.telemetry.addData("h (radians)", odom!!.rOh)
        PanelsTelemetry.telemetry.addData("distance", odom!!.distance)

        PanelsTelemetry.telemetry.addData("CM", CommandManager.snapshot.toString());
        PanelsTelemetry.telemetry.update();
    }
}
