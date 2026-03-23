package org.firstinspires.ftc.teamcode.opmodes.testing.baseSubsystems

import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition

@Configurable
@TeleOp(name = "Servo Turret Test", group = "Base Subsystem Tests")
class ServoTurretTest : NextFTCOpMode() {
    companion object {
        @JvmField var endPosOther = 0.98375;
        @JvmField var servoPos = 0.0;
    }

    val servoAbove = ServoEx("turret_above")  // 0.0
    val servoBelow = ServoEx("turret_below")  // 0.99

    override fun onStartButtonPressed() {

    }

    override fun onUpdate() {
        SetPosition(servoAbove, servoPos)()
        SetPosition(servoBelow, endPosOther - servoPos)()
    }
}