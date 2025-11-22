package org.firstinspires.ftc.teamcode.opmodes.testing

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.driving.FieldCentric
import dev.nextftc.hardware.driving.MecanumDriverControlled
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.subsystems.LowerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometrySubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.MagazineServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.MagblockServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.PusherServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.hypot

@TeleOp(name = "Tele Op In Prog")
@Configurable
class TeleOpInProg : NextFTCOpMode() {
    companion object {
        var overaimSecs = 0.0;
//        var speed = 1234.0;
        var goalX = 12.0;
        var goalY = 144.0-12.0;
        var isBlue = true;
        var startFar = true;
        var ODOM_SHIFT_SPEED = 0.1;
    }
//    private var odom: LocalizationSubsystem? = null
    private var odom: OdometrySubsystem? = null;

    private val lfw = MotorEx("lfw").reversed();
    private val lbw = MotorEx("lbw").reversed();
    private val rfw = MotorEx("rfw");
    private val rbw = MotorEx("rbw");

    private var magDrive: Command? = null;
    private var intakeDrive: Command? = null;

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

//    var ballCnt = 0;
    override fun onStartButtonPressed() {
        var startX: Double;
        var startY: Double;
        if (isBlue) {
            if (startFar) {
                startX = 24.0*3.0-16;
                startY = 24.0*3.0-61;
            } else {
                startX = 24.0*3.0-16;
                startY = 24.0*3.0+61;
            }
        } else {
            if (startFar) {
                startX = 24.0*3.0+16;
                startY = 24.0*3.0-61;
            } else {
                startX = 24.0*3.0+16;
                startY = 24.0*3.0+61;
            }
        }

        val dd = true;  // dual driver?

        // Initialize the device
        odom!!.setPinpoint(startX, startY, -Math.PI / 2.0);

        // Push towards goal
//        var xs = 0.0;
//        var ys = 0.0;
//        Gamepads.gamepad1.dpadUp whenTrue { ys -= ODOM_SHIFT_SPEED }
//        Gamepads.gamepad1.dpadDown whenTrue { ys += ODOM_SHIFT_SPEED }
//        Gamepads.gamepad1.dpadLeft whenTrue { xs -= ODOM_SHIFT_SPEED }
//        Gamepads.gamepad1.dpadRight whenTrue { xs += ODOM_SHIFT_SPEED }
//        if (xs != 0.0 || ys != 0.0) {
//
//        }

        // Reset Odom
        if (dd) (Gamepads.gamepad2.leftBumper and Gamepads.gamepad2.rightBumper)
        else (Gamepads.gamepad1.leftBumper and Gamepads.gamepad1.rightBumper)
            .whenBecomesTrue {
                odom!!.setPinpoint(startX, startY, -Math.PI / 2.0);
            }

        intakeDrive = IntakeSubsystem.DriverCommand(
            if (dd) Gamepads.gamepad2.rightTrigger else Gamepads.gamepad1.rightTrigger,
            if (dd) Gamepads.gamepad2.leftTrigger else Gamepads.gamepad1.leftTrigger
        );
        intakeDrive!!.schedule();


        magDrive = MagazineServoSubsystem.DriverCommandDefaultOn(
            if (dd) Gamepads.gamepad2.leftTrigger else Gamepads.gamepad1.leftTrigger
        );
        magDrive!!.schedule();

        Gamepads.gamepad2.circle or Gamepads.gamepad1.circle
            .whenBecomesTrue(MagblockServoSubsystem.open)
        Gamepads.gamepad2.circle or Gamepads.gamepad1.circle
            .whenBecomesFalse(MagblockServoSubsystem.close)
        Gamepads.gamepad1.cross whenBecomesTrue PusherServoSubsystem.`in`
        Gamepads.gamepad1.cross whenBecomesFalse PusherServoSubsystem.out

        val mecanum = MecanumDriverControlled(
            lfw,
            rfw,
            lbw,
            rbw,
            -Gamepads.gamepad1.leftStickY,
            Gamepads.gamepad1.leftStickX,
            Gamepads.gamepad1.rightStickX,
            FieldCentric({
                if (isBlue) (odom!!.rOh - PI).rad else (odom!!.rOh).rad
            })
        )
        mecanum();



        val thetaAim = TurretThetaSubsystem.AutoAim(
            {
                hypot(
                    goalX - odom!!.rOx1 + odom!!.vx * overaimSecs,
                    goalY - odom!!.rOy1 + odom!!.vy * overaimSecs,
                )
            },
            { (-0.058 * it + 63.0).coerceIn(55.0, 63.0).deg }
        )
        thetaAim();


        val autoAimPhi = TurretPhiSubsystem.AutoAim(
            { goalX - odom!!.rOx1 + odom!!.vx * overaimSecs },
            { goalY - odom!!.rOy1 + odom!!.vy * overaimSecs },
            { odom!!.rOh.rad + odom!!.omega.rad * overaimSecs }
        );
        autoAimPhi.schedule();



//        Gamepads.gamepad1.square whenBecomesTrue SequentialGroup(
//            LowerSubsystem.launchAll,
//            magDrive!!,
//            intakeDrive!!
//        )

        val shooterAutoAim = ShooterSubsystem.AutoAim(
            { hypot(goalX - odom!!.rOx1, goalY - odom!!.rOy1) },
            { (912 + 1.85*it + 0.0148*it*it).coerceIn(0.0, 1800.0) }
        )
        shooterAutoAim.schedule();


        MagblockServoSubsystem.close()
        PusherServoSubsystem.out()
    }

//    var onCmd: ShooterSubsystem.On? = null;
    override fun onUpdate() {
        odom!!.updateOdom()

//        if (onCmd != null) CommandManager.cancelCommand(onCmd!!)
//        val cmd = ShooterSubsystem.On(speed);
//        cmd();
//        onCmd = cmd;

        PanelsTelemetry.telemetry.addData("ang degs", (atan2(goalY - odom!!.rOy1, goalX - odom!!.rOx1).rad - odom!!.rOh.rad).inDeg)

        PanelsTelemetry.telemetry.addData("x (inch)", odom!!.rOx1)
        PanelsTelemetry.telemetry.addData("y (inch)", odom!!.rOy1)
        PanelsTelemetry.telemetry.addData("h (radians)", odom!!.rOh)
        PanelsTelemetry.telemetry.addData("distance", odom!!.distance)

        PanelsTelemetry.telemetry.addData("CM", CommandManager.snapshot.toString());
        PanelsTelemetry.telemetry.update();
    }
}
