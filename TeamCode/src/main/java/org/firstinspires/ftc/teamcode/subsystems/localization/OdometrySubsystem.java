package org.firstinspires.ftc.teamcode.subsystems.localization;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.INCH;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

public class OdometrySubsystem {

  private GoBildaPinpointDriver pinpoint;

  // robot pose values (in inches / radians)
  private double ROx1;
  private double ROy1;
  private double ROh;
  private double distance;

  public OdometrySubsystem(double x1, double y1, double h, HardwareMap hm) {
    if (hm == null) {
      throw new IllegalArgumentException("HardwareMap is null — OpMode may not be initialized yet.");
    }

    // Try to get the GoBilda Pinpoint hardware safely
    try {
      pinpoint = hm.get(GoBildaPinpointDriver.class, "pinpoint");
    } catch (Exception e) {
      pinpoint = null;
      e.printStackTrace();
    }

    if (pinpoint == null) {
      throw new IllegalArgumentException("Pinpoint device not found in HardwareMap. Check DS config name!");
    }

    // Initialize the device
    pinpoint.resetPosAndIMU();
    pinpoint.recalibrateIMU();

    Pose2D initPose = new Pose2D(INCH, x1, y1, AngleUnit.RADIANS, h);

    // Configure sensors and set initial position
    pinpoint.setOffsets(1, 3, INCH);
    pinpoint.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.REVERSED
    );
    pinpoint.setPosition(initPose);
    pinpoint.update();

    // Set initial values
    updateOdom();
  }

  public void updateOdom() {
    if (pinpoint == null) return;

    pinpoint.update();
    ROx1 = pinpoint.getPosition().getX(INCH);
    ROy1 = pinpoint.getPosition().getY(INCH);
    ROh = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);
    distance = Math.hypot(ROx1, ROy1);
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
  public double getDistance() { return distance; }
}
