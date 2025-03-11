package org.firstinspires.ftc.teamcode.example.kotlin

import com.rowanmcalpin.nextftc.core.Subsystem
import com.rowanmcalpin.nextftc.core.command.Command
import com.rowanmcalpin.nextftc.core.command.utility.InstantCommand
import com.rowanmcalpin.nextftc.core.control.controllers.PIDFController
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.MotorEx
import com.rowanmcalpin.nextftc.ftc.hardware.controllables.RunToPosition

object Lift: Subsystem() {
    lateinit var motor: MotorEx

    val controller = PIDFController(0.005, 0.0, 0.0)

    val resetZero: Command
        get() = InstantCommand({ motor.resetEncoder() })

    val name = "lift_motor"

    val toLow: Command
        get() = RunToPosition(motor, // MOTOR TO MOVE
            0.0, // TARGET POSITION, IN TICKS
            controller, // CONTROLLER TO IMPLEMENT
            this) // IMPLEMENTED SUBSYSTEM

    val toMiddle: Command
        get() = RunToPosition(motor, // MOTOR TO MOVE
            500.0, // TARGET POSITION, IN TICKS
            controller, // CONTROLLER TO IMPLEMENT
            this) // IMPLEMENTED SUBSYSTEM

    val toHigh: Command
        get() = RunToPosition(motor, // MOTOR TO MOVE
            1200.0, // TARGET POSITION, IN TICKS
            controller, // CONTROLLER TO IMPLEMENT
            this) // IMPLEMENTED SUBSYSTEM

    override fun initialize() {
        motor = MotorEx(name)
    }
}
