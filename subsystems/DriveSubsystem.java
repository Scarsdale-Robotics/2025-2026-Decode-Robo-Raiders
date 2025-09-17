package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

public class DriveSubsystem implements Subsystem {
  private final MotorEx leftFront;
  private final MotorEx rightFront;
  private final MotorEx leftBack;
  private final MotorEx rightBack;

  public double driveSpeed;
  public double driveTheta;
  public double turnVelocity;

  public double targetX;
  public double targetY;
  public double targetH;
  public DriveSubsystem(MotorEx leftFront,
                        MotorEx rightFront,
                        MotorEx leftBack,
                        MotorEx rightBack) {
    this.leftFront = leftFront;
    this.rightFront = rightFront;
    this.leftBack = leftBack;
    this.rightBack = rightBack;

    this.driveSpeed = 0;
    this.driveTheta = 0;
    this.turnVelocity = 0;
    this.targetX = 0;
    this.targetY = 0;
    this.targetH = 0;
  }


  /////////////
  // GETTERS //
  /////////////

  public double getLeftFrontPosition() {
    return -leftFront.getCurrentPosition();
  }

  public double getRightFrontPosition() {
    return -rightFront.getCurrentPosition();
  }

  public double getLeftBackPosition() {
    return leftBack.getCurrentPosition();
  }

  public double getRightBackPosition() {
    return -rightBack.getCurrentPosition();
  }
  ///////////////////
  // DRIVE METHODS //
  ///////////////////

  /**
   * Drives with directions based on robot pov.
   *
   * @param strafe     Strafe power.
   * @param forward     Forward power.
   * @param turn      Turn power.
   */
  public void driveRobotCentricPowers(double strafe, double forward, double turn) {

  }

  /**
   * Drives with directions based on robot pov.
   *
   * @param theta     Direction of drive in radians.
   * @param speed     Desired driving speed in in/s.
   * @param turn      Desired angular velocity in rad/s.
   */
  public void driveRobotCentric(double theta, double speed, double turn) {
    driveFieldCentric(theta, speed, -turn, 0.0);
  }

  /**
   * Drives based on driver pov.
   * @param gyroAngle Robot heading in radians.
   */
  public void driveFieldCentric(double gyroAngle) {
    driveFieldCentric(driveTheta, driveSpeed, turnVelocity, gyroAngle);
  }

  /**
   * Drives based on driver pov.
   *
   * @param strafe     Strafe power.
   * @param forward     Forward power.
   * @param turn      Turn power.
   * @param gyroAngle Robot heading in radians.
   */
  public void driveFieldCentricPowers(double strafe, double forward, double turn, double gyroAngle) {
  }

  /**
   * Corrected driving with bias based on driver pov.
   *
   * @param theta     Direction of drive in radians.
   * @param speed     Desired driving speed in in/s.
   * @param turn      Desired angular velocity in rad/s.
   * @param gyroAngle Robot heading in radians.
   */
  public void driveFieldCentric(double theta, double speed, double turn, double gyroAngle) {
    theta = normalizeAngle(Math.PI/2 + (theta-gyroAngle));
    double maxSpeed = Math.hypot(
        40.0*Math.cos(theta),//Max strafe speed
        54.0*Math.sin(theta)//Max forward speed
    );

    double L = 0;
    double R = 0;

    double theta_w =
  }

  /**
   * Stop the motors.
   */
  public void stopController() {

  }


  /**
   * Normalizes a given angle to (-pi,pi] radians.
   * @param radians the given angle in radians.
   * @return the normalized angle in radians.
   */
  private static double normalizeAngle(double radians) {

  }

  /**
   * Normalize the wheel speeds
   */
  private void normalize(double[] wheelSpeeds) {


  }


  /////////////////////////////////////
  // AUTOMATIC DRIVING FUNCTIONALITY //
  /////////////////////////////////////
  public void stepTowards2DPoint(Point error) {
    // TODO: IMPLEMENT
  }

  public void stepTowardsFieldLocation() {
    // TODO: IMPLEMENT
  }
}
