package org.firstinspires.ftc.teamcode.subsystems.outtake;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class ShooterSubsystem implements Subsystem {
  public static final ShooterSubsystem INSTANCE = new ShooterSubsystem();

  private final MotorEx shootMotor = new MotorEx("Shooter");

  private ShooterSubsystem() {

  }

  public Command shoot = new LambdaCommand()
    .setStart(() -> shootMotor.setPower(1))
    .setInterruptible(true)
    .setRequirements(ShooterSubsystem.INSTANCE);

  public Command stopShoot = new LambdaCommand()
    .setStart(() -> shootMotor.setPower(0))
    .setRequirements(ShooterSubsystem.INSTANCE);



}
