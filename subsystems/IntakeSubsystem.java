package org.firstinspires.ftc.teamcode.subsystems;

import java.util.function.Supplier;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class IntakeSubsystem implements Subsystem {
    public static final IntakeSubsystem INSTANCE = new IntakeSubsystem();
    public MotorEx intakeMotor = new MotorEx("intakeMotor");
    private IntakeSubsystem() { }

    public Command setPower(Supplier<Double> leftTrigger, Supplier<Double> rightTrigger) {
        return new InstantCommand(() ->
                intakeMotor.setPower(rightTrigger.get() - leftTrigger.get()));
    }

    public Command setPower(double power) {
        return new InstantCommand(() ->
                intakeMotor.setPower(power));
    }

    public Command in = new SetPower(intakeMotor, 1);// power val is placeholder

    public Command out = new SetPower(intakeMotor, -1);// power val is placeholder
    public Command stop = new SetPower(intakeMotor, 1);// power val is placeholder
}
