package org.firstinspires.ftc.teamcode.opmodes.testing

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
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
import kotlin.math.floor
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

        var ODOM_OFS_IPS = 1.0;  // Odom offsetting (adjusting) inches per second
        var ODOM_OFS_RPS = PI/32.0;  // Odom offsetting (adjusting) radians per second

        val dd = true;  // true -> dual driver, false -> single driver
    }
//    private var odom: LocalizationSubsystem? = null
    private var odom: OdometrySubsystem? = null;

    private val lfw = MotorEx("lfw").reversed();
    private val lbw = MotorEx("lbw").reversed();
    private val rfw = MotorEx("rfw");
    private val rbw = MotorEx("rbw");

    private var magDrive: Command? = null;
    private var intakeDrive: Command? = null;

    private var ofsX = 0.0;
    private var ofsY = 0.0;
    private var ofsH = 0.0;

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

    val YELLOW = Triple(255.0, 255.0, 0.0);
    val RED = Triple(255.0, 0.0, 0.0);
    val GREEN = Triple(0.0, 255.0, 0.0);
    val WHITE = Triple(255.0, 255.0, 255.0);

    fun setGamepadColors(r: Double, g: Double, b: Double) {
        gamepad1.setLedColor(r, g, b, Gamepad.LED_DURATION_CONTINUOUS)
        gamepad2.setLedColor(r, g, b, Gamepad.LED_DURATION_CONTINUOUS)
    }

    fun setGamepadColors(rgb: Triple<Double, Double, Double>) {
        setGamepadColors(rgb.first, rgb.second, rgb.third);
    }

    override fun onInit() {
//        odom = LocalizationSubsystem(0.0, 0.0, 0.0, isBlue, hardwareMap)
        odom = OdometrySubsystem(0.0, 0.0, 0.0, hardwareMap)
        setGamepadColors(YELLOW)
    }

//    var ballCnt = 0;
    override fun onStartButtonPressed() {
//        setGamepadColors(WHITE)

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

        // Initialize the device
        odom!!.setPinpoint(startX, startY, -Math.PI / 2.0);

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
                odom!!.setPinpoint(24.0*3, 24.0*2, -Math.PI / 2.0);
            }

        intakeDrive = IntakeSubsystem.DriverCommand(
            if (dd) Gamepads.gamepad2.rightTrigger else Gamepads.gamepad1.rightTrigger,
            if (dd) Gamepads.gamepad2.leftTrigger else Gamepads.gamepad1.leftTrigger,
//            { setGamepadColors(GREEN) },
//            { setGamepadColors(RED) },
//            { setGamepadColors(WHITE) },
        );
        intakeDrive!!.schedule();


        magDrive = MagazineServoSubsystem.DriverCommandDefaultOn(
            if (dd) Gamepads.gamepad2.leftTrigger else Gamepads.gamepad1.leftTrigger
        );
        magDrive!!.schedule();

        // could do the if (dd) s below but lazy hehehehe
        Gamepads.gamepad2.circle whenBecomesTrue MagblockServoSubsystem.open
        Gamepads.gamepad2.circle whenBecomesFalse MagblockServoSubsystem.close
        Gamepads.gamepad2.cross whenBecomesTrue PusherServoSubsystem.`in`
        Gamepads.gamepad2.cross whenBecomesFalse PusherServoSubsystem.out

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
    var lastRuntime = 0.0;
    override fun onUpdate() {
        val dt = runtime - lastRuntime;
        lastRuntime = runtime;

        setGamepadColors(hsvToRgb(runtime * 120.0 % 360.0, 1.0, 1.0));

        odom!!.updateOdom()
        if (dd) {
            if (isBlue) {
                ofsX += gamepad2.left_stick_y * ODOM_OFS_IPS * dt
                ofsY += gamepad2.left_stick_x * ODOM_OFS_IPS * dt
                ofsH += gamepad2.right_stick_x * ODOM_OFS_RPS * dt
            } else {
                ofsX += -gamepad2.left_stick_y * ODOM_OFS_IPS * dt
                ofsY += gamepad2.left_stick_x * ODOM_OFS_IPS * dt
                ofsH += gamepad2.right_stick_x * ODOM_OFS_RPS * dt
            }
        }

//        if (onCmd != null) CommandManager.cancelCommand(onCmd!!)
//        val cmd = ShooterSubsystem.On(speed);
//        cmd();
//        onCmd = cmd;

        PanelsTelemetry.telemetry.addData("ang degs", (atan2(goalY - odom!!.rOy1, goalX - odom!!.rOx1).rad - odom!!.rOh.rad).inDeg)

        PanelsTelemetry.telemetry.addData("x (inch)", odom!!.rOx1 + ofsX)
        PanelsTelemetry.telemetry.addData("y (inch)", odom!!.rOy1 + ofsY)
        PanelsTelemetry.telemetry.addData("h (degrees)", odom!!.rOh.rad.inDeg + ofsH.rad.inDeg)

        PanelsTelemetry.telemetry.addData("CM", CommandManager.snapshot.toString());
        PanelsTelemetry.telemetry.update();

        telemetry.addData("X POS (inch)", odom!!.rOx1 + ofsX)
        telemetry.addData("Y POS (inch)", odom!!.rOy1 + ofsY)
        telemetry.addData("H (degrees)", odom!!.rOh.rad.inDeg + ofsH.rad.inDeg)

        telemetry.update()
    }

    fun hsvToRgb(h: Double, s: Double, v: Double): Triple<Double, Double, Double> {
        if (s == 0.0) {
            // Achromatic (grey)
            return Triple(v, v, v)
        }

        val hue = h / 60.0
        val i = floor(hue).toInt()
        val f = hue - i
        val p = v * (1.0 - s)
        val q = v * (1.0 - s * f)
        val t = v * (1.0 - s * (1.0 - f))

        val r: Double
        val g: Double
        val b: Double

        when (i) {
            0 -> { r = v; g = t; b = p }
            1 -> { r = q; g = v; b = p }
            2 -> { r = p; g = v; b = t }
            3 -> { r = p; g = q; b = v }
            4 -> { r = t; g = p; b = v }
            else -> { r = v; g = p; b = q } // Case 5 and any potential default
        }

        return Triple(r, g, b)
    }
}
