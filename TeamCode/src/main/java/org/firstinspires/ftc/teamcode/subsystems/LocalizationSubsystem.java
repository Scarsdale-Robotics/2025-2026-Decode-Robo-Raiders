package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.localization.CVSubsystem_VisionPortal;
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
    CVSubsystem_VisionPortal cv;
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

    public LocalizationSubsystem(double x1, double y1, double h, HardwareMap hm) {
        cv = new CVSubsystem_VisionPortal(x1, y1, h, hm);
        odom = new OdometrySubsystem(x1, y1, h, hm);
        kalmanX = new Kalman(x1, 0.05, 0.3, 0.7);// higher q = less trust in odom | higher r = less trust in cam
        kalmanY = new Kalman(y1, 0.05, 0.3, 0.7); // higher q = less trust in odom | higher r = less trust in cam
        Rx = kalmanX.getEstimate();
        Ry = kalmanY.getEstimate();
        Rh = odom.getROh();
        clock = System.currentTimeMillis();
        lastUpdateTime = clock;
        timeSinceLastUpdate = 0;
    }

    public void updateLocalization() {
        clock = System.currentTimeMillis();
        timeSinceLastUpdate = clock - lastUpdateTime;
        lastUpdateTime = clock;

        cv.updateCV();
        odom.updateOdom();



        kalmanX.predict(odom.getROx1());
        kalmanY.predict(odom.getROy1());

        if (cv.hasDetection()) {
          kalmanX.correct(cv.getRCx1());
          kalmanY.correct(cv.getRCy1());
        }

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
        tHistory.add(clock / 1000.0); // might have to convert ms to seconds
    }

    private double computeFirstDerivative(LinkedList<Double> values, LinkedList<Double> times) {
        int n = values.size();
        if (n < 2) return 0.0;

        if (n >= 5) {
            double h = (times.get(4) - times.get(0)) / 4.0;
            return (-values.get(4) + 8 * values.get(3) - 8 * values.get(1) + values.get(0)) / (12 * h);
        }

        double h = (times.get(n - 1) - times.get(0)) / (n - 1);
        return (values.get(n - 1) - values.get(0)) / (times.get(n - 1) - times.get(0));
    }

    private double computeSecondDerivative(LinkedList<Double> values, LinkedList<Double> times) {
        int n = values.size();
        if (n < 3) return 0.0;

        if (n >= 5) {
            double h = (times.get(4) - times.get(0)) / 4.0;
            return (-values.get(4) + 16 * values.get(3) - 30 * values.get(2) + 16 * values.get(1) - values.get(0)) / (12 * h * h);
        }

        double h1 = times.get(1) - times.get(0);
        double h2 = times.get(n - 1) - times.get(n - 2);
        if (n == 3) {
            double h = (times.get(2) - times.get(0)) / 2.0;
            return (values.get(2) - 2 * values.get(1) + values.get(0)) / (h * h);
        }
        double h = (times.get(n - 1) - times.get(0)) / (n - 1);
        return (values.get(n - 1) - 2 * values.get(n / 2) + values.get(0)) / (h * h);
    }

    public void setPos(double x1, double y1, double h) {
        cv.setCv(x1, y1, h);
        odom.setPinpoint(x1, y1, h);
        kalmanX = new Kalman(x1, 0.05, 0.3, 0.7); // higher q = less trust in odom | higher r = less trust in cam
        kalmanY = new Kalman(y1, 0.05, 0.3, 0.7); // higher q = less trust in odom | higher r = less trust in cam
    }


    /// gets///
    public double getX() {
        return Rx;
    }

    public double getY() {
        return Ry;
    }

    public double getH() {
        return Rh;
    }

    public double getVX() {
        return Vx;
    }

    public double getVY() {
        return Vy;
    }

    public double getVH() {
        return Vh;
    }

    public double getAX() {
        return Ax;
    }

    public double getAY() {
        return Ay;
    }

    public double getAH() {
        return Ah;
    }

    public double getTimeSinceLastUpdate() {
        return timeSinceLastUpdate;
    }

    public double getClock() {
        return clock;
    }

    public double getTimeS() {
        return clock / 1000;
    }

    public double getLastUpdateTime() {
        return lastUpdateTime;
    }

    public double getKalmangainX() {
        return kalmanX.getKalmanGain();
    }

    public double getKalmangainY() {
        return kalmanY.getKalmanGain();
    }


    public boolean resetLocalizationFromCamera() {
        // Check if the camera has a valid detection
        if (!cv.hasDetection()) {
            return false; // No tag detected, can't reset
        }

        // Get camera-based position
        double camX = cv.getRCx1();
        double camY = cv.getRCy1();
        double camH = cv.getRCh(); // Make sure CVSubsystem returns heading if available

        // Reset odometry
        odom.setPinpoint(camX, camY, camH);

        // Reset Kalman filters with camera values
        kalmanX = new Kalman(camX, 0.05, 0.3, 0.7);
        kalmanY = new Kalman(camY, 0.05, 0.3, 0.7);

        // Reset stored positions and history
        Rx = camX;
        Ry = camY;
        Rh = camH;

        xHistory.clear();
        yHistory.clear();
        hHistory.clear();
        tHistory.clear();

        // Add initial camera reading to history
        xHistory.add(Rx);
        yHistory.add(Ry);
        hHistory.add(Rh);
        tHistory.add(clock / 1000.0);

        // Reset velocities and accelerations
        Vx = Vy = Vh = 0.0;
        Ax = Ay = Ah = 0.0;

        return true; // Reset successful
    }

}

