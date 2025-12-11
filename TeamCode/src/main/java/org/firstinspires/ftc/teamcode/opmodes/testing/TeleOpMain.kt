package org.firstinspires.ftc.teamcode.opmodes.testing

import com.bylazar.configurables.annotations.Configurable
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.groups.SequentialGroup
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.rad
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.driving.FieldCentric
import dev.nextftc.hardware.driving.MecanumDriverControlled
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.opmodes.testing.TeleOpInProg.Companion.isBlue
import org.firstinspires.ftc.teamcode.subsystems.LowerSubsystem
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometrySubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.IntakeSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.intake.IntakeMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.intake.IntakeServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.MagazineMotorSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.MagblockServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.lower.magazine.PusherServoSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem
import org.firstinspires.ftc.teamcode.utils.Lefile.filePath
import java.io.File
import kotlin.math.PI
import kotlin.math.hypot

@Configurable
open class TeleOpMain(
    private val isRed: Boolean,
    private val goalX: Double,
    private val goalY: Double,
    private val distanceToVelocity: (Double) -> Double,
    private val distanceToTheta: (Double) -> Angle
) : NextFTCOpMode() {

    private var odom: OdometrySubsystem? = null;

    private val lfw = MotorEx("lfw").reversed();
    private val lbw = MotorEx("lbw").reversed();
    private val rfw = MotorEx("rfw");
    private val rbw = MotorEx("rbw");

    private var ofsX = 0.0;
    private var ofsY = 0.0;
    private var ofsH = 0.0;

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

    var speedFactor = 1.0;

    override fun onInit() {
        addComponents(
            SubsystemComponent(
                LowerSubsystem,
                OuttakeSubsystem
            ),
            BindingsComponent,
            BulkReadComponent
        )

        ShooterSubsystem.off()
        MagazineMotorSubsystem.off()
        IntakeMotorSubsystem.off()

        odom = OdometrySubsystem(0.0, 0.0, 0.0, hardwareMap)
    }

    override fun onStartButtonPressed() {
        IntakeServoSubsystem.up()
        PusherServoSubsystem.out()
        MagblockServoSubsystem.close()

        // DRIVER CONTROLS
        // Drivetrain
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

        // Scoring
        Gamepads.gamepad1.circle whenBecomesTrue SequentialGroup(
            MagblockServoSubsystem.open,
            MagazineMotorSubsystem.On(FAST_SPEED)
        )


        // AUTO AIM
        ShooterSubsystem.AutoAim(
            { hypot(goalX - x, goalY - y) },
            distanceToVelocity
        )
        TurretPhiSubsystem.AutoAim(
            { goalX - x },
            { goalY - y },
            { h }
        )
        TurretThetaSubsystem.AutoAim(
            { hypot(goalX - x, goalY - y) },
            distanceToTheta
        )

    }

    override fun onUpdate() {
        odom!!.updateOdom()
    }

    override fun onStop() {
        val file = File(filePath)
        file.writeText(
            x.toString() + "\n" +
                    y.toString() + "\n" +
                    h.inRad.toString() + "\n"
        )
    }

}