package org.firstinspires.ftc.teamcode.subsystems.outtake.turret;

import org.firstinspires.ftc.teamcode.subsystems.outtake.TurretSubsystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;

// up-down
public class TurretThetaSubsystem implements Subsystem {
    public static final TurretThetaSubsystem INSTANCE = new TurretThetaSubsystem();

    private final ServoEx servo = new ServoEx("Servo Turret Theta");
    public static double MIN_SERVO_THETA = 0, MAX_SERVO_THETA = 1;  // TODO: TUNE

    private TurretThetaSubsystem() {
        servo.setPosition(0.0);
    }

    public Command setTheta(double theta) {
        return new InstantCommand(() -> servo.setPosition(
                (theta / (2 * Math.PI) + 0.5) % 1 *
                        (MAX_SERVO_THETA - MIN_SERVO_THETA) + MIN_SERVO_THETA
        ));
    }

    public boolean atTarget() { return true; }
}
