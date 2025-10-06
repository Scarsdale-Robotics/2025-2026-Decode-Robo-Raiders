package org.firstinspires.ftc.teamcode.subsystems.localization;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

import dev.nextftc.core.subsystems.Subsystem;

public class OdometrySubsystem {


  public double ROx1;
  public double ROy1;
  public double ROh; //normalized to [pi, -pi)
  public double distance;
  public GoBildaPinpointDriver pinpoint;

  public OdometrySubsystem(double x1, double y1, double h, HardwareMap hm){
    pinpoint.update();
    pinpoint.getPosY();
  }
}
