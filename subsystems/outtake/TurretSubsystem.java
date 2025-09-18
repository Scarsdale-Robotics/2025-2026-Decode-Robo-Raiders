package org.firstinspires.ftc.teamcode.subsystems.outtake;

import dev.nextftc.core.subsystems.Subsystem;

public class TurretSubsystem implements Subsystem {
    public static final TurretSubsystem INSTANCE = new TurretSubsystem();

    public double target_x;
    public double target_y;
    private TurretSubsystem() { }

    public void autoTarget(){

    }

    public void turretSetGoal(double x, double y){
      target_x = x;
      target_y = y;
    }

    public void turretSetGoalX(double x){
      target_x = x;
    }

    public void turretSetGoalY(double y){
      target_y = y;
    }
    public void onUpdate(){
      // move towards targets
    }
    public void turretSetPower(double x, double y){
      // direct set motors
    }


}
