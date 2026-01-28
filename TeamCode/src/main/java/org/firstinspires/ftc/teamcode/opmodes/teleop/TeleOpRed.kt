package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import kotlin.math.PI
import kotlin.math.max
import kotlin.math.min

@TeleOp(name = "[RED] Tele Op")
class TeleOpRed : TeleOpBase(
    isBlue = false,
    goalX = 144.0 - 3.0,
    goalY = 144.0 - 3.0,
    resetModeParams = ResetModeParams(144.0 - 8.127, 6.5, (PI / 2.0).rad),
    distanceToVelocity = { 0.0127 * it * it + 1.81 * it + 937.0 },
    distanceToTheta = { max(min(-0.224*it+74, 63.0), 55.0).deg },
    distanceToTime = { 0.0 }  // todo: tune  // is over seconds, not milliseconds
)
