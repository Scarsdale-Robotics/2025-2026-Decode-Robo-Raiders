package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class IntakeSubsystem implements Subsystem {
    public static final IntakeSubsystem INSTANCE = new IntakeSubsystem();
    public MotorEx intakeMotor = new MotorEx("intakeMotor");
    private IntakeSubsystem() { }

    public Command in = new SetPower(intakeMotor, 1);// power val is placeholder

    public Command out = new SetPower(intakeMotor, -1);// power val is placeholder
    public Command stop = new SetPower(intakeMotor, 1);// power val is placeholder
}
