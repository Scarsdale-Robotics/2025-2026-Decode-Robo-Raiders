package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.driving.FieldCentric
import dev.nextftc.hardware.driving.MecanumDriverControlled
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.pedroPathing.Constants.createFollower
import org.firstinspires.ftc.teamcode.subsystems.LowerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometrySubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.MagblockServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem
import java.util.function.Supplier
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

        @JvmField var goalX = 3.0;
        @JvmField var goalY = 144.0 - 3.0;
    }

    init {
        addComponents(
            SubsystemComponent(
                LowerSubsystem,
                OuttakeSubsystem,
                MagServoSubsystem,
                MagblockServoSubsystem,
                TurretThetaSubsystem,
                TurretPhiSubsystem
            ),
            PedroComponent(Constants::createFollower),
            BindingsComponent,
            BulkReadComponent
        )
    }


    val x:  Double get() { return (PedroComponent.follower.pose.x);}
    val y:  Double get() { return (PedroComponent.follower.pose.y);}
    val h:  Angle  get() { return (PedroComponent.follower.pose.heading).rad;}

//    private var odom: OdometrySubsystem? = null;
    override fun onInit() {
        ShooterSubsystem.off()
        MagMotorSubsystem.off()
        MagServoSubsystem.stop()
//        odom = OdometrySubsystem(72.0, 72.0, -PI / 2, hardwareMap)
    }

    var speedFactor = 1.0;
    override fun onStartButtonPressed() {
        PedroComponent.follower.pose = Pose(72.0, 72.0, -PI / 2)
        MagblockServoSubsystem.unblock()
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

        val lowerMotorDrive = MagMotorSubsystem.DriverCommand(
            Gamepads.gamepad1.rightTrigger,
            Gamepads.gamepad1.leftTrigger,
            { 0.0 }
        );
        lowerMotorDrive();

        val intakeMotorDrive = IntakeMotorSubsystem.DriverCommand(
            Gamepads.gamepad1.rightTrigger,
            Gamepads.gamepad1.leftTrigger,
            { 0.0 }
        )
        intakeMotorDrive()

//        val magDrive = MagServoSubsystem.DriverCommandDefaultOn(
//            Gamepads.gamepad1.leftTrigger.greaterThan(0.0)
//        )
//        magDrive();

        Gamepads.gamepad1.circle
            .whenTrue(MagblockServoSubsystem.unblock)
            .whenBecomesFalse(MagblockServoSubsystem.block)

    }

    override fun onUpdate() {
        PedroComponent.follower.update()
        TurretPhiSubsystem.AutoAim(
            goalX - x, goalY - y, h
        )()
        TurretThetaSubsystem.SetTargetTheta(shootAngleDegrees.deg)()
        ShooterSubsystem.On(speed1)();
        telemetry.addData("x (inch)", x);
        telemetry.addData("y (inch)", y);
        telemetry.addData("h (radians)", h);
        telemetry.addData(
            "distanceToGoal",
            hypot((3 - x), (141 - y))
        );
        telemetry.addData("ShooterSpeed", speed1);
        telemetry.addData("Angle", shootAngleDegrees.deg);
        telemetry.update()
        PanelsTelemetry.telemetry.update()
    }

}
