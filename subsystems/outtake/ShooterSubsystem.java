package org.firstinspires.ftc.teamcode.subsystems.outtake;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class ShooterSubsystem implements Subsystem {
    public static final ShooterSubsystem INSTANCE = new ShooterSubsystem();

    private final MotorEx ShootMotor = new MotorEx("Shooter Motor");
    private ShooterSubsystem() {

    }
    public boolean isOn(){
      return true;
    };

    /**
   * sets whether we want open or closed
   * @param goalstate true for open, false for closed
   * @return
   */
    public void setOn(boolean goalstate){

    }

    public void onUpdate(){
      // maybe this does something
    }

}
