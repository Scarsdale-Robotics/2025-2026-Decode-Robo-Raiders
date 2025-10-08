package org.firstinspires.ftc.teamcode.opmodes.testing;

import static org.firstinspires.ftc.teamcode.utils.QuarticMaxNonnegRoot.maxNonNegativeRoot;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.OuttakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.outtake.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.outtake.TurretSubsystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

public class T67 extends NextFTCOpMode {

    private final MotorEx frontLeftMotor = new MotorEx("front_left").reversed();
    private final MotorEx frontRightMotor = new MotorEx("front_right");
    private final MotorEx backLeftMotor = new MotorEx("back_left").reversed();
    private final MotorEx backRightMotor = new MotorEx("back_right");

    private double backupFarPhi = 0;  // TODO: Tune
    private double backupClosePhi = 0;
    private double backupFarTheta = 0;
    private double backupCloseTheta = 0;

    public T67() {
        addComponents(
                new SubsystemComponent(
                        IntakeSubsystem.INSTANCE,
                        LocalizationSubsystem.INSTANCE,
                        OuttakeSubsystem.INSTANCE
                )
        );
    }

    private final int TILE_SIZE = 24;
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

        CommandManager.INSTANCE.scheduleCommand(
                TurretSubsystem.INSTANCE.autoAim(telemetry)
        );

        // circle is shoot
        Gamepads.gamepad1().triangle().and(Gamepads.gamepad1().leftBumper().not())
                .whenBecomesTrue(OuttakeSubsystem.INSTANCE.shootWhenReady)
                .whenBecomesFalse(ShooterSubsystem.INSTANCE.stopShoot);
        // circle and left bumper is force shoot
        Gamepads.gamepad1().triangle().and(Gamepads.gamepad1().leftBumper())
                .whenBecomesTrue(ShooterSubsystem.INSTANCE.startShoot)
                .whenBecomesFalse(ShooterSubsystem.INSTANCE.stopShoot);

        // pedro coords
        // intended reset point, for now: 24,24 (audience-side, blue, margin 1, robot center)
        // reset coords
        Gamepads.gamepad1().leftBumper().and(Gamepads.gamepad1().rightBumper())
                .whenTrue(
                        new ParallelGroup(
                                LocalizationSubsystem.INSTANCE.resetH(0),
                                LocalizationSubsystem.INSTANCE.resetX(1 * TILE_SIZE),
                                LocalizationSubsystem.INSTANCE.resetY(1 * TILE_SIZE)
                        )
                );

        // INTAKE
        Command intakePower = IntakeSubsystem.INSTANCE.setPower(
                Gamepads.gamepad1().leftTrigger(),
                Gamepads.gamepad1().rightTrigger()
        );
        CommandManager.INSTANCE.scheduleCommand(intakePower);

        //         //
        // BACKUPS //
        //         //

        Gamepads.gamepad2().dpadDown()
                .whenTrue(
                        TurretSubsystem.INSTANCE.setAim(
                                backupFarTheta,
                                backupFarPhi
                        )
                )
    }

}
