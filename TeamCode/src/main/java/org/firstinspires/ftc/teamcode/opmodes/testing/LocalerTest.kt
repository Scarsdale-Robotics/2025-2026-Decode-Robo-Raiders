package org.firstinspires.ftc.teamcode.opmodes.testing

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import dev.nextftc.core.commands.CommandManager
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.hardware.driving.MecanumDriverControlled
import dev.nextftc.hardware.impl.MotorEx

@TeleOp(name = "LocalerTest")
class LocalerTest : NextFTCOpMode() {
    init {
        addComponents(
            BulkReadComponent,
            BindingsComponent
        )
    }

    val frontLeft = MotorEx("frontLeft")
    val frontRight = MotorEx("frontRight").reversed()
    val backLeft = MotorEx("backLeft")
    val backRight = MotorEx("backRight").reversed()

    override fun onStartButtonPressed() {
        val driverControlled = MecanumDriverControlled(
             frontLeft,
            frontRight,
            backLeft,
            backRight,
            Gamepads.gamepad1.leftStickY,
            Gamepads.gamepad1.leftStickX,
            Gamepads.gamepad1.rightStickX
        )
        driverControlled()
    }

    override fun onUpdate() {
        super.onUpdate()
        telemetry.addData("CMD", CommandManager.snapshot)
        telemetry.update()
    }
}
