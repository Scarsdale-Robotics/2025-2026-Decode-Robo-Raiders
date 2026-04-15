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
    goalX = 144.0 - 6.0,
    goalY = 144.0 - 6.0,
    resetModeParams = ResetModeParams(144.0-17.1887, 115.3623, 270.0.deg),
//    resetModeParams = ResetModeParams(8.127, 6.5, (PI / 2.0).rad),
//    resetModeParams = ResetModeParams(8.127, 5.0, (PI / 2.0).rad),
    resetModePhiAngle = 0.0.deg,
    distanceToVelocityClose = { d -> AutoAimConstants.distanceToVelocityClose(d + D_OFS) },
    distAndVeloToThetaClose = { d, v -> AutoAimConstants.distAndVeloToNewThetaClose(d + D_OFS, v) },
    distanceToVelocityFar = { d -> AutoAimConstants.distanceToVelocityFar(d + D_OFS) },
    distAndVeloToThetaFar = { d, v -> AutoAimConstants.distAndVeloToNewThetaFar(d + D_OFS, v) },
    distanceToTimeClose = { d -> AutoAimConstants.distanceToTimeClose(d + D_OFS) },
    distanceToTimeFar = { d -> AutoAimConstants.distanceToTimeFar(d + D_OFS) }
)
