package org.firstinspires.ftc.teamcode.subsystems.outtake;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;

public class ShooterSubsystem implements Subsystem {
    public static final ShooterSubsystem INSTANCE = new ShooterSubsystem();
    private ShooterSubsystem() {

    }
    public boolean isopen(){
      return true;
    };

    /**
   * sets whether we want open or closed
   * @param goalstate true for open, false for closed
   * @return
   */
    public void setgoal(boolean goalstate){

    }

    public void runUpdate(){
      //open or close based on goal
    }

}
