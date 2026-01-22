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
  private double distance;

//  private Supplier<Double> ofsXSupplier, ofsYSupplier;
//  private double ofsX, ofsY;

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
    pinpoint.setOffsets(-96, -25, MM);
    pinpoint.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.FORWARD
    );
    pinpoint.setPosition(initPose);

    // Set initial values
    updateOdom();
  }

//  public void setOffsetSuppliers(Supplier<Double> ofsXSupplier, Supplier<Double> ofsYSupplier) {
//    this.ofsXSupplier = ofsXSupplier;
//    this.ofsYSupplier = ofsYSupplier;
//  }

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

    distance = Math.hypot(ROx1, ROy1);  // dist a little misleading: this is not dist travelled, this is dist from (0, 0)
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

  public double getDistance() { return distance; }
}
