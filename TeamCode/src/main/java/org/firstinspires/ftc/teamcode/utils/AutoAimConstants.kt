package org.firstinspires.ftc.teamcode.utils

import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.deg
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sqrt

object AutoAimConstants {
    const val BORD_Y = 48;

    fun distanceToVelocityClose(d: Double): Double {
        return 715 + 9.25 * d + -0.0353 * d * d;
    }

    fun distAndVeloToNewThetaClose(d: Double, v: Double): Double {
        return max(min(1.69 + -0.0203 * d + 9.73E-05 * d * d, 0.9), 0.63)
    };

    fun distanceToVelocityFar(d: Double): Double {
        return 2685 + -23.6 * d + 0.109 * d * d
    }

    fun distAndVeloToNewThetaFar(d: Double, v: Double): Double {
        return 0.63
    };

    // first ball time (seconds)
    fun distanceToTimeClose(d: Double): Double {
        return (-9.99E-03 + 0.0113 * d + -5.29E-05 * d * d) + 0.188
    }

    // first ball time
    fun distanceToTimeFar(d: Double): Double {
        return (2.43 + -0.0304 * d + 1.3E-04 * d * d) + 0.188
    }



    ////////////////////////





    // todo: redo
    fun distAndVeloToThetaFar(d: Double, v: Double): Angle {
//        val (b0, b1, b2, b3, b4) = arrayOf(
//            0.0132524, 0.000895715, -0.000145183, -5.41118, 0.351313
//        );
//        val b5 = 168.64602
//        return max(min((b0*d*d+b1*d*v+b2*v*v+b3*d+b4*v+b5), 63.0), 55.0).deg;

        val b0 = 117.08157;
        val b1 = -0.195638;
        val b2 = -645202.755;
        val b3 = 1192.83621;

        if (v == 0.0) return 63.0.deg;

        return max(min((
                b0 +
                        b1 * d +
                        b2 * d / (v * v) +
                        b3 * (d * d) / (v * v)
                ), 63.0), 55.0).deg;

//        val (b0, b1, b2, b3, b4) = arrayOf(
//            195.7715, -0.709742, 515380256.0, -9231672.5, 35387.6019
//        );
//        return max(min((
//                b0 +
//                b1 * d +
//                b2 / (v * v) +
//                b3 * d / (v * v) +
//                b4 * (d * d) / (v * v)
//        ), 63.0), 55.0).deg;
    }

    fun distAndVeloToThetaClose(d: Double, v: Double): Angle {
        val b0 = 70.90073
        val b1 = -0.366685
        val b2 = 7519899.79
        val b3 = -225684.275
        val b4 = 4969.34854

        if (v == 0.0) return 63.0.deg;

        return max(min((
                b0 +
                        b1 * d +
                        b2 / (v * v) +
                        b3 * d / (v * v) +
                        b4 * (d * d) / (v * v)
                ), 63.0), 55.0).deg;
    }

}