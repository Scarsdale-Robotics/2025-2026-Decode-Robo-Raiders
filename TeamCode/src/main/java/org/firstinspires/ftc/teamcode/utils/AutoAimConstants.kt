package org.firstinspires.ftc.teamcode.utils

import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.deg
import kotlin.math.ln
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.sqrt

object AutoAimConstants {
    fun distanceToVelocityClose(d: Double): Double {
        val (b0, b1, b2) = arrayOf(
            -609.4652, 304.04375, -11.0255
        );
        return b0 + b1 * sqrt(d) + b2 * d;
    }

    fun distAndVeloToThetaClose(d: Double, v: Double): Angle {
        val (b0, b1, b2, b3, b4) = arrayOf(
            77.50779, -0.429656, 4507231.16, -263782.983, 5613.57621
        );
        return max(min((
                b0 +
                b1 * d +
                b2 / (v * v) +
                b3 * d / (v * v) +
                b4 * (d * d) / (v * v)
        ), 63.0), 55.0).deg;
    }

    fun distanceToVelocityFar(d: Double): Double {
        val (b0, b1, b2) = arrayOf(
            2907.55323, -437.74778, 26.97694
        );
        return b0 + b1 * sqrt(d) + b2 * d;
    }

    fun distAndVeloToThetaFar(d: Double, v: Double): Angle {
        val (b0, b1, b2, b3, b4) = arrayOf(
            195.7715, -0.709742, 515380256.0, -9231672.5, 35387.6019
        );
        return max(min((
                b0 +
                b1 * d +
                b2 / (v * v) +
                b3 * d / (v * v) +
                b4 * (d * d) / (v * v)
        ), 63.0), 55.0).deg;
    }

    // first ball
    fun distanceToTime(d: Double): Double {
        val (b0, b1) = arrayOf(-0.77681, 0.158807)
        return b0 + b1 * sqrt(d)
    }
}