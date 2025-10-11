package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.subsystems.localization.CVSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.localization.Kalman;
import org.firstinspires.ftc.teamcode.subsystems.localization.OdometrySubsystem;

import java.util.LinkedList;

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
  CVSubsystem cv;
  OdometrySubsystem odom;
  Kalman kalmanX;
  Kalman kalmanY;

  //timing stuff
  double clock;
  double timeSinceLastUpdate;
  double lastUpdateTime;

  //5 point stencil stuff
  private final LinkedList<Double> xHistory = new LinkedList<>();
  private final LinkedList<Double> yHistory = new LinkedList<>();
  private final LinkedList<Double> hHistory = new LinkedList<>();
  private final LinkedList<Double> tHistory = new LinkedList<>();

  public LocalizationSubsystem(double x1, double y1, double h, boolean side, HardwareMap hm) {
    cv = new CVSubsystem(x1, y1, h, side, hm);
    odom = new OdometrySubsystem(x1, y1, h, hm);
    kalmanX = new Kalman(x1, 0.05, 0.3, 0.7);// tune - (0.3q = 70% trust odom)
    kalmanY = new Kalman(y1, 0.05, 0.3, 0.7);// tune - (0.3q = 70% trust odom
    Rx = kalmanX.getEstimate();
    Ry = kalmanY.getEstimate();
    Rh = odom.getROh();
    clock = System.currentTimeMillis();
    lastUpdateTime = clock;
    timeSinceLastUpdate = 0;
  }

  public void update() {
    clock = System.currentTimeMillis();
    timeSinceLastUpdate = clock - lastUpdateTime;
    lastUpdateTime = clock;
    cv.updateCV();
    odom.updateOdom();
    kalmanX.updateKF(odom.getROx1(), cv.getRCx1());
    kalmanY.updateKF(odom.getROy1(), cv.getRCy1());
    Rx = kalmanX.getEstimate();
    Ry = kalmanY.getEstimate();
    Rh = odom.getROh();
    updateHistory();
    Vx = computeFirstDerivative(xHistory, tHistory);
    Vy = computeFirstDerivative(yHistory, tHistory);
    Vh = computeFirstDerivative(hHistory, tHistory);
    Ax = computeSecondDerivative(xHistory, tHistory);
    Ay = computeSecondDerivative(yHistory, tHistory);
    Ah = computeSecondDerivative(hHistory, tHistory);
  }

  private void updateHistory() {
    if (xHistory.size() >= 5) {
      xHistory.removeFirst();
      yHistory.removeFirst();
      hHistory.removeFirst();
      tHistory.removeFirst();
    }
    xHistory.add(Rx);
    yHistory.add(Ry);
    hHistory.add(Rh);
    tHistory.add(clock); // might have to convert ms to seconds
  }

  private double computeFirstDerivative(LinkedList<Double> values, LinkedList<Double> times) {
    if (values.size() < 5) return 0.0;
    double h = (times.get(4) - times.get(0)) / 4.0;
    return (-values.get(4) + 8*values.get(3) - 8*values.get(1) + values.get(0)) / (12*h);
  }

  private double computeSecondDerivative(LinkedList<Double> values, LinkedList<Double> times) {
    if (values.size() < 5) return 0.0;
    double h = (times.get(4) - times.get(0)) / 4.0;
    return (-values.get(4) + 16*values.get(3) - 30*values.get(2) + 16*values.get(1) - values.get(0)) / (12*h*h);
  }

  public double getX() { return Rx; }
  public double getY() { return Ry; }
  public double getH() { return Rh; }
  public double getVX() { return Vx; }
  public double getVY() { return Vy; }
  public double getVH() { return Vh; }
  public double getAX() { return Ax; }
  public double getAY() { return Ay; }
  public double getAH() { return Ah; }
  public double getTimeSinceLastUpdate() { return timeSinceLastUpdate; }
  public double getClock() { return clock; }
  private double getTimeS() { return System.nanoTime() / 1_000_000.0; }
  public void setPos(double x1, double y1, double h) { cv.setCv(x1, y1, h); odom.setPinpoint(x1, y1, h); }
  public CVSubsystem.motif getMotif() { return cv.getMotifc(); }
}
