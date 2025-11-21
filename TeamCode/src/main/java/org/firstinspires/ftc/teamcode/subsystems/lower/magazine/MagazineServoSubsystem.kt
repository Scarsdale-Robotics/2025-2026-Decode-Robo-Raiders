package org.firstinspires.ftc.teamcode.subsystems.lower.magazine

import com.acmerobotics.dashboard.config.Config
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.CRServoEx
import dev.nextftc.hardware.powerable.SetPower
import java.util.function.Supplier

@Config
object MagazineServoSubsystem : Subsystem {
    @JvmField var FORWARD = 1.0;
    @JvmField var REVERSE = -1.0;

    private val servoL = CRServoEx("mwl");
    private val servoR = CRServoEx("mwr");

    val forward = ParallelGroup(
        SetPower(servoL, -FORWARD),
//        SetPower(servoR, FORWARD)
    );
    val reverse = ParallelGroup(
        SetPower(servoL, -REVERSE),
//        SetPower(servoR, REVERSE)
    );
    val stop = ParallelGroup(
        SetPower(servoL, -0.0),
        SetPower(servoR, 0.0)
    );

    class DriverCommandDefaultOn(  // could be bad for power draw?
        private val outPower: Supplier<Double>,
    ) : Command() {
        override val isDone = false;

        init {
//            setInterruptible(true);
            setRequirements(MagazineServoSubsystem);
        }

        override fun update() {
            val power =  1.0 - 2.0 * outPower.get();
            servoL.power = -power;
//            servoR.power = power;
        }
    }

    class DriverCommand(
        private val inPower: Supplier<Double>,
        private val outPower: Supplier<Double>,
    ) : Command() {
        override val isDone = false;

        init {
            setRequirements(MagazineServoSubsystem);
        }

        override fun update() {
            val power = inPower.get() - outPower.get();
            servoL.power = -power;
//            servoR.power = power;
        }
    }
}
