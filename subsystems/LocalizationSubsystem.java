package org.firstinspires.ftc.teamcode.subsystems;


import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.localization.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometrySubsystem;

import dev.nextftc.core.subsystems.SubsystemGroup;

public class LocalizationSubsystem extends SubsystemGroup {

  private double x;
  private double y;
  private double h;



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



  public double getX(){
    return x; // make this work eventually
  }
  public double getY(){
    return y; // make this work eventually
  }
  public double getH(){
    return h; // make this work eventually
  }
  public void resetH(double H){

  }

  @Override
  public void periodic(){
    // update headings based on locale modules
  }
}
