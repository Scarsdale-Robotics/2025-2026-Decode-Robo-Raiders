package org.firstinspires.ftc.teamcode.opmodes.testing;

import static org.firstinspires.ftc.teamcode.utils.Angles.normalizeAngle;
import static org.firstinspires.ftc.teamcode.utils.QuarticMaxNonnegRoot.maxNonNegativeRoot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.outtake.TurretSubsystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "Auto Aim Test")
public class AutoAimTest extends NextFTCOpMode {

    private final MotorEx frontLeftMotor = new MotorEx("front_left").reversed();
    private final MotorEx frontRightMotor = new MotorEx("front_right");
    private final MotorEx backLeftMotor = new MotorEx("back_left").reversed();
    private final MotorEx backRightMotor = new MotorEx("back_right");

    private final int TILE_SIZE = 24;
    private final int FIELD_SIZE = 6 * TILE_SIZE;

    public static boolean isRed = false;

    private final double g = 9.80665;
    private final double v0 = 5;  // TODO: TEST, tune, check if angle changes significantly, etc.

    public AutoAimTest() {
        addComponents(
                new SubsystemComponent(
                        IntakeSubsystem.INSTANCE,
                        LocalizationSubsystem.INSTANCE,
                        OuttakeSubsystem.INSTANCE
                )
        );
    }

    @Override
    public void onStartButtonPressed() {
        Command driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
        );
        driverControlled.schedule();

        // cross is auto-aim
        Gamepads.gamepad1().cross()
                .whenTrue(autoAim);

        // triangle is shoot
        Gamepads.gamepad1().triangle().and(Gamepads.gamepad1().leftBumper().not())
                .whenBecomesTrue(OuttakeSubsystem.INSTANCE.shootWhenReady)
                .whenBecomesFalse(ShooterSubsystem.INSTANCE.stopShoot);
        // triangle and left bumper is force shoot
        Gamepads.gamepad1().triangle().and(Gamepads.gamepad1().leftBumper())
                .whenBecomesTrue(ShooterSubsystem.INSTANCE.startShoot)
                .whenBecomesFalse(ShooterSubsystem.INSTANCE.stopShoot);

        // pedro coords
        // intended reset point, for now: 24,24 (audience-side, blue, margin 1, robot center)
        Gamepads.gamepad1().leftBumper().and(Gamepads.gamepad1().rightBumper())
                .whenTrue(
                        new ParallelGroup(
                                LocalizationSubsystem.INSTANCE.resetH(0),
                                LocalizationSubsystem.INSTANCE.resetX(1 * TILE_SIZE),
                                LocalizationSubsystem.INSTANCE.resetY(1 * TILE_SIZE)
                        )
                );
    }


    private Telemetry.Item autoAimTime = telemetry.addData("[ATA] t", "...");
    private Telemetry.Item autoAimThetaGoal = telemetry.addData("[ATA] θ", "...");
    private Telemetry.Item autoAimPhiGoal = telemetry.addData("[ATA] φ", "...");

    private Command autoAim = new InstantCommand(() -> {
        // TARGET INFO (T = Target)
        double xT = isRed ? (FIELD_SIZE - TILE_SIZE / 2.0) : (TILE_SIZE / 2.0);
        double yT = TILE_SIZE / 2.0;
        double zT = 42.0;  // 38.75 is min, 53.75 is max

        // TURRET INFO (U = Turret)
        double xU = LocalizationSubsystem.INSTANCE.getX() + 0;  // TODO: add turret offsets
        double yU = LocalizationSubsystem.INSTANCE.getY() + 0;
        double zU = 5.0;

        // ROBOT INFO (R = Robot)
        double hR = LocalizationSubsystem.INSTANCE.getH();

        double vxU = LocalizationSubsystem.INSTANCE.getVX();  // more representative of vxR, but close enough
        double vyU = LocalizationSubsystem.INSTANCE.getVY();
        double vUmag = Math.hypot(vxU, vyU);

        if (Math.abs(vxU) > 5 || Math.abs(vyU) > 5) {
            telemetry.addLine("(!) [ATA] Velocity spike (" + vxU + " m/s, " + vyU + " m/s)");
        }

        // Deltas
        double dz = zT - zU;
        double dy = yT - yU;
        double dx = xT - xU;
        double drMag = Math.hypot(dx, dy);

        double[] coeffs = {
                0.25 * g * g,
                0,
                vUmag * vUmag + g * dz - v0 * v0,
                -2 * drMag * vUmag * Math.cos(normalizeAngle(Math.atan2(dy, dx) - Math.atan2(vyU, vxU))),
                Math.pow(drMag, 2)
        };
        double t = maxNonNegativeRoot(coeffs);

        if (t == Double.NaN) {
            telemetry.addLine("(!) [ATA] No positive time found");
        }

        // note notation opposite Stephen's doc, equal to subsystems, phi l/r, theta up/down
        double theta_res = Math.asin((dz + 0.5 * g * t * t) / (v0 * t));
        double phi_res = Math.atan2(dy - vyU * t, dx - vxU * t);

        telemetry.addData("[ATA] t", "%.3f s", t);
        telemetry.addData("[ATA] θ", "%.2f deg", Math.toDegrees(theta_res));
        telemetry.addData("[ATA] φ", "%.2f deg", Math.toDegrees(phi_res));

        telemetry.update();

        TurretSubsystem.INSTANCE.setAim(theta_res, phi_res);
    });
}
