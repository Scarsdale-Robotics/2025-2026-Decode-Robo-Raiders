package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.subsystems.localization.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.localization.Kalman;
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometrySubsystem;


public class LocalizationSubsystem {

  double Rx;
  double Ry;
  double Rh;
  double Vx;
  double Vy;
  double Vh;
  double Ax;
  double Ay;
  double Ah;
  CVSubsystem cv; //note rember to have a method to return mosique
  OdometrySubsystem odom;
  Kalman kalmanX;
  Kalman kalmanY;

  public LocalizationSubsystem(double x1, double y1, double h, boolean side, HardwareMap hm){
    cv = new CVSubsystem(x1, y1, h, side, hm);
    odom = new OdometrySubsystem(x1, y1, h, hm);
    kalmanX = new Kalman(x1, 0.05, 0.3, 0.7); //tune - (0.3q = 70% trust odom)
    kalmanY = new Kalman(y1,0.05, 0.3, 0.7 ); //tune - (0.3q = 70% trust odom)
    Rx = kalmanX.getEstimate();
    Ry = kalmanY.getEstimate();
    Rh = odom.getROh();
  }

  public void update(){
    cv.updateCV();
    odom.updateOdom();
    kalmanX.updateKF(odom.getROx1(), cv.getRCx1());
    kalmanY.updateKF(odom.getROy1(), cv.getRCy1());
    Rx = kalmanX.getEstimate();
    Ry = kalmanY.getEstimate();
    Rh = odom.getROh();
  }






  ///gets/setters///
  public double getX() { return Rx; }
  public double getY() { return Ry; }
  public double getH() { return Rh; }
  public double getVX() { return 0.0; }
  public double getVY() { return 0.0; }
  public double getVH() { return 0.0; }
  public double getAX() { return 0.0; }
  public double getAY() { return 0.0; }
  public double getAH() { return 0.0; }
  public void setPos(double x1, double y1, double h){cv.setCv(x1, y1, h); odom.setPinpoint(x1, y1, h);}
  public CVSubsystem.motif getMotif() {return cv.getMotifc();}

}