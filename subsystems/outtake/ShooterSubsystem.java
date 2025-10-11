package org.firstinspires.ftc.teamcode.subsystems.outtake;

import org.firstinspires.ftc.teamcode.utils.SMO.SMOFilter;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.builder.FilterBuilder;
import dev.nextftc.control.feedback.FeedbackType;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class ShooterSubsystem implements Subsystem {
    public static final ShooterSubsystem INSTANCE = new ShooterSubsystem();

    private final MotorEx shootMotor = new MotorEx("Shooter");

    private double targetVelocity = 1.0;

    public static double Ls = 0.0, Lv = 0.0, La = 0.0, kSqu = 0.0, kI = 0.0, kD = 0.0;
    private ControlSystem controlSystem;

    private ShooterSubsystem() {
        SMOFilter velSMO = new SMOFilter(FeedbackType.VELOCITY, Ls, Lv, La);
        controlSystem = ControlSystem.builder()
                .velFilter(filter -> filter.custom(velSMO))
                .velSquID(new PIDCoefficients(kSqu, kI, kD))
                .build();
    }

    @Override
    public void periodic() {
        shootMotor.setPower(
                controlSystem.calculate(
                    new KineticState(0, targetVelocity)  // todo: if not work, consider pos != 0
                )
        );
    }

    public Command setVelocity(double targetVelocity) {
        return new InstantCommand(() -> this.targetVelocity = targetVelocity);
    }

}
