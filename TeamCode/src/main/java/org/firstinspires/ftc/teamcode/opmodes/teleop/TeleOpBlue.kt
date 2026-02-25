package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import java.lang.Math.pow
import kotlin.math.PI
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
        -0.120936 * dist * dist + 24.33705 * dist + 71.32418
    },
    distAndVeloToThetaClose = { dist, velo -> max(min(
        -0.00130433 * dist * dist +
                -0.00000964864 * velo * velo +
                0.0116661 * dist +
                -0.0249701 * velo +
                0.000230304 * dist * velo +
                85.88317,
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
    distanceToTime = { 0.0 }  // todo: tune  // is over seconds, not milliseconds
//    distanceToTime = { -0.739 + 0.0185 * it + -4.6E-05 * it * it }
)
