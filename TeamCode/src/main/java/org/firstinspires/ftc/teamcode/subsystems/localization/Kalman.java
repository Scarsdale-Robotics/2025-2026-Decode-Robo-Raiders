package org.firstinspires.ftc.teamcode.subsystems.localization;

public class Kalman {
    private double x;       // current estimate (fused value)
    private double p;       // error covariance (uncertainty in estimate)
    private double q;       // process noise (prediction uncertainty) — closer to 0 means more trust in prediction
    private double r;       // measurement noise (sensor uncertainty) — closer to 0 means more trust in measurement
    private double k;       // Kalman gain — closer to 1 means more trust in measurement, closer to 0 means more trust in prediction


    public Kalman(double initialEstimate, double initialP, double q, double r) {
        this.x = initialEstimate;
        this.p = initialP;
        this.q = q;
        this.r = r;
    }

    public void predict(double delta) {
        if (!Double.isFinite(delta)) return;
        x += delta;
        p += q;
    }

    public void correct(double measurement) {
        if (!Double.isFinite(measurement)) return;
        k = p / (p + r);
        x = x + k * (measurement - x);
        p = (1 - k) * p;
    }

    public void predictAngle(double delta) {
        if (!Double.isFinite(delta)) return;
        x += delta;
        //[-pi, pi]
        while (x > Math.PI) x -= 2 * Math.PI;
        while (x < -Math.PI) x += 2 * Math.PI;
        p += q;
    }

    public void correctAngle(double measurement) {
        if (!Double.isFinite(measurement)) return;
        k = p / (p + r);
        double diff = measurement - x;
        while (diff > Math.PI) diff -= 2 * Math.PI;
        while (diff < -Math.PI) diff += 2 * Math.PI;
        x = x + k * diff;
        while (x > Math.PI) x -= 2 * Math.PI;
        while (x < -Math.PI) x += 2 * Math.PI;
        p = (1 - k) * p;
    }

    public double getEstimate() {
        return x;
    }

    public double getKalmanGain() {
        return k;
    }

    public double getErrorCovariance() {
        return p;
    }

    public void setMeasurementNoise(double r) {
        this.r = r;
    }
}


