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

    public double updateKF(double predection, double measurement) {
        //arjan are we serious u cant even spell prediction what is predection bro
        x += predection;
        p += q;
        k = p / (p + r);
        x = x + k * (measurement - x);
        p = (1 - k) * p;
        return x;
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
}
