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
    goalX = 3.0,
    goalY = 144.0 - 6.0,
    resetModeParams = ResetModeParams(144.0 - 8.127, 6.5, (PI / 2.0).rad),
    resetModePhiAngle = (-180.0).deg,
    distanceToVelocityClose = { d -> AutoAimConstants.distanceToVelocityClose(d) },
    distAndVeloToThetaClose = { d, v -> AutoAimConstants.distAndVeloToThetaClose(d, v) },
    distanceToVelocityFar = { d -> AutoAimConstants.distanceToVelocityFar(d) },
    distAndVeloToThetaFar = { d, v -> AutoAimConstants.distAndVeloToThetaFar(d, v) },
//    distanceToTheta = { max(min(-0.000153176 * it * it + 0.00180231 * it + 63.63936, 63.0), 55.0).deg },
    distanceToTimeClose = { d -> AutoAimConstants.distanceToTimeClose(d) },
    distanceToTimeFar = { d -> AutoAimConstants.distanceToTimeFar(d) }
//    distanceToTime = { -0.739 + 0.0185 * it + -4.6E-05 * it * it }
)
