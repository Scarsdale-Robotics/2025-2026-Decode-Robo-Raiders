package org.firstinspires.ftc.teamcode.subsystems.localization;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;
import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.MM;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

public class OdometrySubsystem {

  private GoBildaPinpointDriver pinpoint;

  // robot pose values (in inches / radians)
  private double ROx1;
  private double ROy1;
  private double ROh;
  private double Vx, Vy, omega;
  private double distFromOrigin;



  public OdometrySubsystem(double x1, double y1, double h, HardwareMap hm) {
    if (hm == null) {
      throw new IllegalArgumentException("HardwareMap is null — OpMode may not be initialized yet.");
    }

    try {
      pinpoint = hm.get(GoBildaPinpointDriver.class, "pinpoint");
    } catch (Exception e) {
      pinpoint = null;
      e.printStackTrace();
    }

    if (pinpoint == null) {
      throw new IllegalArgumentException("u fucked up");
    }

    // Initialize the device
    pinpoint.resetPosAndIMU();
    pinpoint.recalibrateIMU();

    Pose2D initPose = new Pose2D(INCH, x1, y1, AngleUnit.RADIANS, h);

    // Configure sensors and set initial position

    pinpoint.setOffsets(-24.64271, -95.64271, MM);
    pinpoint.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.FORWARD
    );
    pinpoint.setPosition(initPose);

    // Set initial values
    updateOdom();
  }



  ElapsedTime time = null;
  double rxl = 0.0, ryl = 0.0, rhl = 0.0;
  public void updateOdom() {
    if (pinpoint == null) return;

    pinpoint.update();
    rxl = ROx1; ryl = ROy1; rhl = ROh;
    ROx1 = pinpoint.getPosition().getX(INCH);
    ROy1 = pinpoint.getPosition().getY(INCH);
    ROh = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);

    if (time != null) {
      Vx = (ROx1 - rxl) / time.seconds();
      Vy = (ROy1 - ryl) / time.seconds();
      omega = (ROh - rhl) / time.seconds();

      time.reset();
    } else {
      time = new ElapsedTime();
    }

    distFromOrigin = Math.hypot(ROx1, ROy1);

  }

  public void setPinpoint(double x1, double y1, double h) {
    if (pinpoint == null) return;
    pinpoint.setPosition(new Pose2D(INCH, x1, y1, AngleUnit.RADIANS, h));
  }

  public void resetPinpoint() {
    if (pinpoint == null) return;
    pinpoint.resetPosAndIMU();
    pinpoint.recalibrateIMU();
  }

  // Getters
  public double getROh() { return ROh; }
  public double getROx1() { return ROx1; }
  public double getROy1() { return ROy1; }

  public double getVx() {
    return Vx;
  }

  public double getVy() {
    return Vy;
  }

  public double getOmega() {
    return omega;
  }

  public double getDistFromOrigin() { return distFromOrigin; }
}
