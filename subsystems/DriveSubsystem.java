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
}
