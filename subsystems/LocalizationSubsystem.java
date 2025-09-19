package org.firstinspires.ftc.teamcode.subsystems;


import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.localization.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometrySubsystem;

import dev.nextftc.core.subsystems.SubsystemGroup;

public class LocalizationSubsystem extends SubsystemGroup {
  public static final LocalizationSubsystem INSTANCE = new LocalizationSubsystem();
  private LocalizationSubsystem() {
    super(
      CVSubsystem.INSTANCE,
      OdometrySubsystem.INSTANCE
    );
  }



  public int getPattern(){
    // returns motif pattern
    return 0;
  }

//  public Pose2D getPos(){
//    //gets pos
//    return new Pose2D(getX(), getY(), getH());
//
//  }
  public double getX(){
    return 0; // make this work eventually
  }
  public double getY(){
    return 0; // make this work eventually
  }
  public double getH(){
    return 0; // make this work eventually
  }
  public void resetH(double H){

  }
}
