package org.firstinspires.ftc.teamcode.subsystems.outtake.turret

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import dev.nextftc.core.commands.Command
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPositions
import java.util.function.Supplier
import kotlin.math.max
import kotlin.math.min

// up-down
@Configurable
// to reset servo, place at pos zero, then lay hood on, then inc servo value
object TurretThetaSubsystem : Subsystem {
    private val servo = ServoEx("turret_theta", 0.0001);

    ///0.15 (Highest Angle) 0.75 (Shallow Angle)///
    ///0.15 (Highest Angle) 0.75 (Shallow Angle)///
    ///0.15 (Highest Angle) 0.75 (Shallow Angle)///
    ///0.15 (Highest Angle) 0.75 (Shallow Angle)///
    ///0.15 (Highest Angle) 0.75 (Shallow Angle)///



    class SetThetaPos(val pos: Double) : SetPositions(
        servo to max(min(pos, 0.9), 0.63)
        //im not sure if this work
        //servo to (angle*0.09625 - 5.09375)
    )

    class AutoAim(
        private val dxy: Double,
        private val hoodAngleByDistance: (Double) -> Double,  // get by running curve of best fit on collected data
    ) : Command() {
        override val isDone = true;

        init {
            setName("Auto Aim Theta")
        }

        override fun start() {
            SetThetaPos(hoodAngleByDistance(dxy))()
            PanelsTelemetry.telemetry.addData("s", dxy)
            PanelsTelemetry.telemetry.addData("theta goal", hoodAngleByDistance(dxy))
        }
    }

    @JvmField var thing = 0.005;
    class Manual(
        private val goalChange: Supplier<Double>
    ) : Command() {
        override val isDone = false;

        override fun update() {
            servo.position += goalChange.get() * thing;
        }
    }
}