package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.units.deg
import kotlin.math.max
import kotlin.math.min

@TeleOp(name = "[BLUE] Tele Op")
class TeleOpBlue : TeleOpBase(
    isBlue = true,
    goalX = 3.0,
    goalY = 144.0 - 3.0,
    distanceToVelocity = { 0.0127 * it * it + 1.81 * it + 937.0 },
    distanceToTheta = { max(min(-0.224*it+74, 63.0), 55.0).deg }
)
