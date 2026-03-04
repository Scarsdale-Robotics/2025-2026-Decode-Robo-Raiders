package org.firstinspires.ftc.teamcode.subsystems.localization;

public class Kalman {
    private double x;   // current estimate (fused value)
    private double p;   // error covariance (uncertainty in estimate)
    private double q;   // process noise (prediction uncertainty) — closer to 0 means more trust in prediction
    private double r;   // measurement noise (sensor uncertainty) — closer to 0 means more trust in measurement
    private double k;   // Kalman gain — closer to 1 means more trust in measurement, closer to 0 means more trust in prediction

    public Kalman(double initialEstimate, double initialP, double q, double r) {
        this.x = initialEstimate;
        this.p = initialP;
        this.q = q;
        this.r = r;
        this.k = 0.0;
    }

    public void predict(double delta) {
        if (!Double.isFinite(delta)) return;
        x += delta;
        p += q;
    }

    public void correct(double measurement) {
        if (!Double.isFinite(measurement)) return;
        if (p + r == 0) return;
        k = p / (p + r);
        x = x + k * (measurement - x);
        p = (1 - k) * p;
    }

    public void predictAngle(double delta) {
        if (!Double.isFinite(delta)) return;
        x += delta;
        x = wrapAngle(x);
        p += q;
    }

    public void correctAngle(double measurement) {
        if (!Double.isFinite(measurement)) return;
        if (p + r == 0) return;
        k = p / (p + r);
        double diff = wrapAngle(measurement - x);
        x = wrapAngle(x + k * diff);
        p = (1 - k) * p;
    }

    private double wrapAngle(double angle) {
        while (angle > Math.PI)  angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public double getEstimate()        { return x; }
    public double getKalmanGain()      { return k; }
    public double getErrorCovariance() { return p; }

    public void setMeasurementNoise(double r) { this.r = r; }

    // FIX: added missing setter for process noise — needed if you want to tune q at runtime
    public void setProcessNoise(double q) { this.q = q; }
}