package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.subsystems.localization.CVSubsystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.SubsystemGroup;

// CAUTION !! RESETTING LOCALIZATION SHOULD HANDLE (or at least keep note of) POTENTIAL VELOCITY SPIKE !!
public class LocalizationSubsystem extends SubsystemGroup {

  private double x;
  private double y;
  private double h;



  public static final LocalizationSubsystem INSTANCE = new LocalizationSubsystem();
  private LocalizationSubsystem() {
    super(

    );
  }



  public int getPattern(){
    // returns motif pattern
    return 0;
  }



  public double getXR(){
    return x; // make this work eventually
  }
  public double getYR(){
    return y; // make this work eventually
  }
  public double getHR(){
    return h; // make this work eventually
  }
  public double getVXR(){
    return x; // make this work eventually
  }
  public double getVYR(){
    return y; // make this work eventually
  }
  public double getVHR(){
    return h; // make this work eventually
  }

  public Command resetX(double xR){
    return new InstantCommand(() -> {

    });
  }
  public Command resetY(double yR){
    return new InstantCommand(() -> {

    });
  }
  public Command resetH(double hR){
    return new InstantCommand(() -> {

    });
  }

  @Override
  public void periodic(){
    // update headings based on locale modules
  }
}
