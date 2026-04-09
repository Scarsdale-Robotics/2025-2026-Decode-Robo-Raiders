package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants
import java.lang.Math.pow
import kotlin.math.PI
import kotlin.math.ln
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow

@TeleOp(name = "[BLUE] Tele Op")
class TeleOpBlue : TeleOpBase(
    isBlue = true,
    goalX = 3.5,
    goalY = 144.0 - 3.5,
    resetModeParams = ResetModeParams(17.1887, 115.3623, 270.0.deg),
    resetModePhiAngle = 0.0.deg,
    distanceToVelocityClose = { d -> AutoAimConstants.distanceToVelocityClose(d) },
    distAndVeloToThetaClose = { d, v -> AutoAimConstants.distAndVeloToThetaClose(d, v) },
    distanceToVelocityFar = { d -> AutoAimConstants.distanceToVelocityFar(d) },
    distAndVeloToThetaFar = { d, v -> AutoAimConstants.distAndVeloToThetaFar(d, v) },
//    distanceToTheta = { max(min(-0.000153176 * it * it + 0.00180231 * it + 63.63936, 63.0), 55.0).deg },
    distanceToTimeClose = { d -> AutoAimConstants.distanceToTimeClose(d) },
    distanceToTimeFar = { d -> AutoAimConstants.distanceToTimeFar(d) }
//    distanceToTime = { -0.739 + 0.0185 * it + -4.6E-05 * it * it }
)
