package org.firstinspires.ftc.teamcode.subsystems.outtake;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class ShooterSubsystem implements Subsystem {
  public static final ShooterSubsystem INSTANCE = new ShooterSubsystem();

  private final MotorEx shootMotor = new MotorEx("Shooter Motor");
  private ShooterSubsystem() {

  }


  public void startshoot(){
    shootMotor.setPower(1);
  }
  public void stopshoot(){
    shootMotor.setPower(0);
  }



}
