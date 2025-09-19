package org.firstinspires.ftc.teamcode.subsystems.outtake;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class ShooterSubsystem implements Subsystem {
  public static final ShooterSubsystem INSTANCE = new ShooterSubsystem();

  private final MotorEx shootMotor = new MotorEx("Shooter Motor");
  private ShooterSubsystem() {

  }


  public void startShoot(){
    shootMotor.setPower(1);
  }
  public void stopShoot(){
    shootMotor.setPower(0);
  }



  ////////////////////////
  /// shooter commands ///
  ////////////////////////
  public Command shoot = new LambdaCommand()
    .setStart(ShooterSubsystem.INSTANCE::startShoot)
    .setInterruptible(true)
    .setIsDone(()->false)
    .setRequirements(ShooterSubsystem.INSTANCE);
  public Command stopShoot = new LambdaCommand()
    .setStart(ShooterSubsystem.INSTANCE::stopShoot)
    .setIsDone(()->true)
    .setRequirements(ShooterSubsystem.INSTANCE);



}
