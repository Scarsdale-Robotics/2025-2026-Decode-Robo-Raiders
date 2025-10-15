package org.firstinspires.ftc.teamcode.subsystems.localization;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

public class OdometrySubsystem {


  public double ROx1; // Inch
  public double ROy1; // Inch
  public double ROh; //normalized to [pi, -pi)
  public double distance;
  public GoBildaPinpointDriver pinpoint;

  public OdometrySubsystem(double x1, double y1, double h, HardwareMap hm){
    pinpoint = hm.get(GoBildaPinpointDriver.class,"pinpoint");
    pinpoint.resetPosAndIMU();
    pinpoint.recalibrateIMU();
    Pose2D Init = new Pose2D(INCH, x1,y1,AngleUnit.RADIANS,h);
    pinpoint.update();
    pinpoint.setOffsets(1,3,INCH); //Set properly ltr
    pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED); //Set properly ltr
    pinpoint.setPosition(Init);
    ROx1 = pinpoint.getPosition().getX(INCH);
    ROy1 = pinpoint.getPosition().getY(INCH);
    ROh = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);
    distance = Math.hypot(ROx1, ROy1);
    pinpoint.update();
  }

  public void updateOdom(){
    pinpoint.update();
    ROx1 = pinpoint.getPosition().getX(INCH);
    ROy1 = pinpoint.getPosition().getY(INCH);
    ROh = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);
    distance = Math.hypot(ROx1, ROy1);
  }

  public void setPinpoint(double x1, double y1, double h){
    pinpoint.setPosition(new Pose2D(INCH, x1, y1, AngleUnit.RADIANS, h));
  }

  public void resetPinpoint() {
    pinpoint.resetPosAndIMU();
    pinpoint.recalibrateIMU();
  }



  ///Gets///
  public double getROh() {
    return ROh;
  }
  public double getROx1() {
    return ROx1;
  }
  public double getDistance() {
    return distance;
  }
  public double getROy1() {
    return ROy1;
  }
}

