package org.firstinspires.ftc.teamcode.utils

import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.deg
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

object AutoAimConstants {
    const val BORD_Y = 48;

//    fun distanceToVelocityClose(d: Double): Double {
//        val (b0, b1, b2) = arrayOf(
//            -609.4652, 304.04375, -11.0255
//        );
//        return b0 + b1 * sqrt(d) + b2 * d;
//    }
//
//    fun distAndVeloToThetaClose(d: Double, v: Double): Angle {
//        val (b0, b1, b2, b3, b4) = arrayOf(
//            77.50779, -0.429656, 4507231.16, -263782.983, 5613.57621
//        );
//        return max(min((
//                b0 +
//                b1 * d +
//                b2 / (v * v) +
//                b3 * d / (v * v) +
//                b4 * (d * d) / (v * v)
//        ), 63.0), 55.0).deg;
//    }


    fun distanceToVelocityClose(d: Double): Double {
        val (b0, b1, b2) = arrayOf(
            111.68989, 159.803, -4.00802
        );
        return b0 + b1 * sqrt(d) + b2 * d;
    }

    fun distAndVeloToThetaClose(d: Double, v: Double): Angle {
        val (b0, b1, b2, b3, b4) = arrayOf(
            70.90073, -0.366685, 7519899.79, -225684.275, 4969.34854
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

    // first ball time (seconds)
    fun distanceToTimeClose(d: Double): Double {
        val (b0, b1, b2) = arrayOf(-12.17499, 2.91218, -0.165714)
        return b0 + b1 * sqrt(d) + b2 * d
    }

    // first ball time
    fun distanceToTimeFar(d: Double): Double {
        val (b0, b1, b2) = arrayOf(11.10757, -1.96187, 0.0939321)
        return b0 + b1 * sqrt(d) + b2 * d
    }

}