package org.firstinspires.ftc.teamcode.utils;

public class Angles {
    public static double normalizeAngle(double rads) {
        rads %= 2 * Math.PI;
        if (rads < 0) rads += 2 * Math.PI;
        return rads;
    }
}
