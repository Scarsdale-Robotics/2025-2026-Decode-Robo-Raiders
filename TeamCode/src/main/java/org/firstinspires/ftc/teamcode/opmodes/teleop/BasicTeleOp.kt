package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.follower.Follower
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.driving.FieldCentric
import dev.nextftc.hardware.driving.MecanumDriverControlled
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.opmodes.testing.baseSubsystems.ShooterFTest.Companion.speed
import org.firstinspires.ftc.teamcode.opmodes.testing.multiSubsystem.AutoAimTest.Companion.shootAngleDegrees
import org.firstinspires.ftc.teamcode.subsystems.LowerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometrySubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.LowerMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagblockServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem
import kotlin.math.PI
import kotlin.math.hypot

@Configurable
@TeleOp(name = "Basic TeleOp")
class BasicTeleOp(): NextFTCOpMode() {

    private val lfw = MotorEx("lfw").reversed();
    private val lbw = MotorEx("lbw").reversed();
    private val rfw = MotorEx("rfw");
    private val rbw = MotorEx("rbw");

    companion object {
        @JvmField var speed1 = 0.0;
        @JvmField var shootAngleDegrees = 60;
        @JvmField var isBlue = true;
    }

    val x: Double
        get() {
            if (odom != null) {
                return odom!!.rOx1;
            }
            return 0.0;
        }
    val y: Double
        get() {
            if (odom != null) {
                return odom!!.rOy1;
            }
            return 0.0;
        }
    val h: Angle
        get() {
            if (odom != null) {
                return odom!!.rOh.rad;
            }
            return 0.0.rad;
        }

    private var odom: OdometrySubsystem? = null;
    override fun onInit() {
        addComponents(
            SubsystemComponent(
                LowerSubsystem,
                OuttakeSubsystem,
                MagServoSubsystem,
                MagblockServoSubsystem,
                TurretThetaSubsystem
            ),
            BindingsComponent,
            BulkReadComponent
        )

        ShooterSubsystem.off()
        LowerMotorSubsystem.off()
        MagServoSubsystem.stop()
        odom = OdometrySubsystem(72.0, 72.0, -PI / 2, hardwareMap)
    }

    var speedFactor = 1.0;
    override fun onStartButtonPressed() {
        odom!!.setPinpoint(72.0, 72.0, -PI / 2)
        MagblockServoSubsystem.block()

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

        Gamepads.gamepad1.rightBumper whenBecomesTrue { speedFactor = 0.5; }
        Gamepads.gamepad1.rightBumper whenBecomesFalse { speedFactor = 1.0; }

//        val driveCommand = PedroDriverControlled(
//            Gamepads.gamepad1.leftStickY.map {it},
//            Gamepads.gamepad1.leftStickX.map {it},
//            Gamepads.gamepad1.rightStickX.map {it},
//            false
//        )
//        driveCommand();

        val lowerMotorDrive = LowerMotorSubsystem.DriverCommandDefaultOn(
            Gamepads.gamepad1.leftTrigger
        );
        lowerMotorDrive();

        val magDrive = MagServoSubsystem.DriverCommand(
            Gamepads.gamepad1.leftTrigger.greaterThan(0.0)
        )
        magDrive();

        Gamepads.gamepad1.circle
            .whenTrue(MagblockServoSubsystem.unblock)
            .whenBecomesFalse(MagblockServoSubsystem.block)

    }

    override fun onUpdate() {
        odom!!.updateOdom()
        TurretThetaSubsystem.SetTargetTheta(shootAngleDegrees.deg)()
        ShooterSubsystem.On(speed1)();
        telemetry.addData("x (inch)", odom!!.rOx1);
        telemetry.addData("y (inch)", odom!!.rOy1);
        telemetry.addData("h (radians)", odom!!.rOh);
        telemetry.addData(
            "distanceToGoal",
            hypot((3 - odom!!.rOx1), (141 - odom!!.rOy1))
        );
        telemetry.addData("ShooterSpeed", speed1);
        telemetry.addData("Angle", shootAngleDegrees.deg);
        telemetry.update()
        PanelsTelemetry.telemetry.update()
    }

}
