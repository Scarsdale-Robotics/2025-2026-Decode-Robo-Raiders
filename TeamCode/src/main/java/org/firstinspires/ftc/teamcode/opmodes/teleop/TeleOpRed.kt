package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import org.firstinspires.ftc.teamcode.utils.AutoAimConstants
import kotlin.math.PI
import kotlin.math.ln
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow

@TeleOp(name = "[RED] Tele Op")
class TeleOpRed : TeleOpBase(
    isBlue = false,
    goalX = 144.0 - 3.0,
    goalY = 144.0 - 6.0,
    resetModeParams = ResetModeParams(8.127, 5.0, (PI / 2.0).rad),
    resetModePhiAngle = 180.0.deg,
    distanceToVelocityClose = { d -> AutoAimConstants.distanceToVelocityClose(d) },
    distAndVeloToThetaClose = { d, v -> AutoAimConstants.distAndVeloToThetaClose(d, v) },
    distanceToVelocityFar = { d -> AutoAimConstants.distanceToVelocityFar(d) },
    distAndVeloToThetaFar = { d, v -> AutoAimConstants.distAndVeloToThetaFar(d, v) },
    distanceToTime = { d -> AutoAimConstants.distanceToTime(d) }   // todo: tune  // is over seconds, not milliseconds
)
