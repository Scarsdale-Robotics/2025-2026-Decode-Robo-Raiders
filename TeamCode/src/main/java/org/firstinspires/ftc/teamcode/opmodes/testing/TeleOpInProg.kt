package org.firstinspires.ftc.teamcode.opmodes.testing

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.Angle
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
import java.io.File
import kotlin.math.PI
import kotlin.math.atan2
import kotlin.math.floor
import kotlin.math.hypot

@TeleOp(name = "Tele Op In Prog", group = "Testing")
@Configurable
class TeleOpInProg : NextFTCOpMode() {
    companion object {
        var overaimSecs = 0.0;
        var speed = 1234.0;
        var m: Double? = 0.200;
        var distanceGoalX = 12.0;
        var distanceGoalY = 144.0-12.0;
        var directionGoalX = 4.0;
        var directionGoalY = 144.0-4.0;
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

    var startX: Double = 72.0;
    var startY: Double = 72.0;
    var startH: Double = -PI / 2.0;
    override fun onInit() {
//        odom = LocalizationSubsystem(0.0, 0.0, 0.0, isBlue, hardwareMap)
        val file = File("RobotAutonEndPos.txt")
        val content = file.readText().split("\n")
        startX = content[0].toDouble()
        startY = content[1].toDouble()
        startH = content[2].toDouble()
//        file.writeText(
//            follower!!.pose.x.toString() + "\n" +
//                    follower!!.pose.y.toString() + "\n" +
//                    follower!!.pose.heading.toString())

        odom = OdometrySubsystem(0.0, 0.0, 0.0, hardwareMap)
        setGamepadColors(YELLOW)
    }

    val x: Double
        get() {
            if (odom != null) {
                return odom!!.rOx1 + ofsX;
            }
            return 0.0;
        }
    val y: Double
        get() {
            if (odom != null) {
                return odom!!.rOy1 + ofsY;
            }
            return 0.0;
        }
    val h: Angle
        get() {
            if (odom != null) {
                return odom!!.rOh.rad + ofsH.rad;
            }
            return 0.0.rad;
        }

//    var ballCnt = 0;
    var aac = 0.0;
    var speedFactor = 1.0;
    override fun onStartButtonPressed() {
//        setGamepadColors(WHITE)

    // todo: determine start positions by moving from 72.0, 72.0 to desired start pos

//        if (isBlue) {
//            if (startFar) {
//                startX = 24.0*3.0-16;
//                startY = 24.0*3.0-61;
//            } else {
//                startX = 24.0*3.0-16;
//                startY = 24.0*3.0+61;
//            }
//        } else {
//            if (startFar) {
//                startX = 24.0*3.0+16;
//                startY = 24.0*3.0-61;
//            } else {
//                startX = 24.0*3.0+16;
//                startY = 24.0*3.0+61;
//            }
//        }

        // Initialize the device
        odom!!.setPinpoint(startX, startY, startH);

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

        //// could do the if (dd) s below but lazy hehehehe
        (Gamepads.gamepad1.circle or Gamepads.gamepad2.circle) whenBecomesTrue MagblockServoSubsystem.open
        (Gamepads.gamepad1.circle or Gamepads.gamepad2.circle) whenBecomesFalse MagblockServoSubsystem.close
//        Gamepads.gamepad1.cross whenBecomesTrue { gamepad2.rumble(500) }
//        Gamepads.gamepad2.cross whenBecomesTrue PusherServoSubsystem.`in`
//        Gamepads.gamepad2.cross whenBecomesFalse PusherServoSubsystem.out
        Gamepads.gamepad1.cross whenBecomesTrue PusherServoSubsystem.`in`
        Gamepads.gamepad1.cross whenBecomesFalse PusherServoSubsystem.out

        if (isBlue) {
            Gamepads.gamepad2.dpadUp whenBecomesTrue { ofsX += 4.0; }
            Gamepads.gamepad2.dpadRight whenBecomesTrue { ofsY -= 4.0; }
            Gamepads.gamepad2.dpadLeft whenBecomesTrue { ofsY += 4.0; }
            Gamepads.gamepad2.dpadDown whenBecomesTrue { ofsX -= 4.0; }
        } else {
            Gamepads.gamepad2.dpadUp whenBecomesTrue { ofsX -= 4.0; }
            Gamepads.gamepad2.dpadRight whenBecomesTrue { ofsY += 4.0; }
            Gamepads.gamepad2.dpadLeft whenBecomesTrue { ofsY -= 4.0; }
            Gamepads.gamepad2.dpadDown whenBecomesTrue { ofsX += 4.0; }
        }

        Gamepads.gamepad2.rightBumper whenBecomesTrue { aac += 5.0; }
        Gamepads.gamepad2.leftBumper whenBecomesTrue { aac -= 5.0; }

    // todo: auto aim with velo test
        val mecanum = MecanumDriverControlled(
            lfw,
            rfw,
            lbw,
            rbw,
            -Gamepads.gamepad1.leftStickY.map { it*speedFactor },
            Gamepads.gamepad1.leftStickX.map { it*speedFactor },
            Gamepads.gamepad1.rightStickX.map { it*speedFactor },
            FieldCentric({
                if (isBlue) (h.inRad - PI).rad else h
            })
        )
        mecanum();



        val thetaAim = TurretThetaSubsystem.AutoAim(
            {
                hypot(
                    distanceGoalX - x + odom!!.vx * overaimSecs,
                    distanceGoalY - y + odom!!.vy * overaimSecs,
                )
            },
            { (-m!!*it+70.67).coerceIn(55.0, 63.0).deg }
        )
        thetaAim();


        val autoAimPhi = TurretPhiSubsystem.AutoAim(
            { directionGoalX - x + odom!!.vx * overaimSecs },
            { directionGoalY - y + odom!!.vy * overaimSecs },
            { h + odom!!.omega.rad * overaimSecs }
        );
        autoAimPhi.schedule();


        Gamepads.gamepad1.rightBumper whenBecomesTrue { speedFactor = 0.5; }
        Gamepads.gamepad1.rightBumper whenBecomesFalse { speedFactor = 1.0; }
//        Gamepads.gamepad1.square whenBecomesTrue SequentialGroup(
//            LowerSubsystem.launchAll,
//            magDrive!!,
//            intakeDrive!!
//        )

        val shooterAutoAim = ShooterSubsystem.AutoAim(
            { hypot(distanceGoalX - x, distanceGoalY - y) },
            { (aac + 578 + 12.7*it + -0.0921*it*it + 0.000316*it*it*it).coerceIn(0.0, 1500.0) }
        )
        shooterAutoAim.schedule();


        MagblockServoSubsystem.close()
        PusherServoSubsystem.out()
    }

    var onCmd: ShooterSubsystem.On? = null;
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

        if (onCmd != null) CommandManager.cancelCommand(onCmd!!)
        val cmd = ShooterSubsystem.On(speed);
        cmd();
        onCmd = cmd;

        PanelsTelemetry.telemetry.addData("ang degs", (atan2(distanceGoalY - y, distanceGoalX - x).rad - h).inDeg)

        PanelsTelemetry.telemetry.addData("x (inch)", x)
        PanelsTelemetry.telemetry.addData("y (inch)", y)
        PanelsTelemetry.telemetry.addData("h (degrees)", h)

        PanelsTelemetry.telemetry.addData("CM", CommandManager.snapshot.toString());
        PanelsTelemetry.telemetry.update();

        telemetry.addLine("LOCALIZATION")
        telemetry.addData("X POS (inch)", x)
        telemetry.addData("Y POS (inch)", y)
        telemetry.addData("H (degrees)", h)

        telemetry.addData("distance to goal (inches)", hypot(distanceGoalX - x, distanceGoalY - y))

        telemetry.addLine("\nINPUTS")
        telemetry.addData("speed factor", speedFactor)
        telemetry.addData("velocity auto aim correctional constant, aac (tps)", aac)

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
            0 -> {
                r = v; g = t; b = p
            }

            1 -> {
                r = q; g = v; b = p
            }

            2 -> {
                r = p; g = v; b = t
            }

            3 -> {
                r = p; g = q; b = v
            }

            4 -> {
                r = t; g = p; b = v
            }

            else -> {
                r = v; g = p; b = q
            } // Case 5 and any potential default
        }

        return Triple(r, g, b)
    }
}
