package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import java.lang.Math.pow
import kotlin.math.PI
import kotlin.math.ln
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow

@TeleOp(name = "[BLUE] Tele Op")
class TeleOpBlue : TeleOpBase(
    isBlue = true,
    goalX = 3.0,
    goalY = 144.0 - 6.0,
    resetModeParams = ResetModeParams(144.0 - 8.127, 6.5, (PI / 2.0).rad),
    resetModePhiAngle = (-180.0).deg,
    distanceToVelocityClose = { dist ->
        -0.016772 * dist.pow(2) +
                7.82068 * dist +
                638.1726
    },
    distAndVeloToThetaClose = { dist, velo -> max(min(
        0.000217156 * dist.pow(3) +
                -0.000104074 * dist.pow(2) * velo +
                0.000015072 * dist * velo.pow(2) +
                -6.84068 * 10.0.pow(-7.0) * velo.pow(3) +
                0.0655632 * dist.pow(2) +
                -0.0168192 * dist * velo +
                0.00103424 * velo.pow(2) +
                3.92855 * dist +
                -0.44738 * velo +
                118.82247,
        63.0
    ), 55.0).deg },
    distanceToVelocityFar = { dist ->
        0.0382154 * dist * dist - 2.23133 * dist + 1067.7389
    },
    distAndVeloToThetaFar = { dist, velo -> max(min(
        -0.00351837 * dist * dist +
                -0.000247344 * velo * velo +
                -3.33946 * dist +
                0.381546 * velo +
                0.00275402 * dist * velo +
                2.82046,
        63.0
    ), 55.0).deg},
//    distanceToTheta = { max(min(-0.000153176 * it * it + 0.00180231 * it + 63.63936, 63.0), 55.0).deg },
    distanceToTime = { -2.44 + 0.706 * ln(it) }  // todo: tune  // is over seconds, not milliseconds
//    distanceToTime = { -0.739 + 0.0185 * it + -4.6E-05 * it * it }
)
