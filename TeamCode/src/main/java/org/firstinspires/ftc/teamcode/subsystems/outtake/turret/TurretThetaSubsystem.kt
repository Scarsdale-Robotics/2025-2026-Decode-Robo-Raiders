package org.firstinspires.ftc.teamcode.subsystems.outtake.turret

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import dev.nextftc.bindings.Button
import dev.nextftc.control.KineticState
import dev.nextftc.core.commands.Command
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.hardware.controllable.RunToState
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition
import dev.nextftc.hardware.positionable.SetPositions
//import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem.controller
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem.targetPhi
import java.util.function.Supplier
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin
import kotlin.time.ComparableTimeMark
import kotlin.time.DurationUnit
import kotlin.time.TimeSource

// up-down
@Configurable
// to reset servo, place at pos zero, then lay hood on, then inc servo value
object TurretThetaSubsystem : Subsystem {
    private val servo = ServoEx("turret_theta");

    ///0.15 (Highest Angle) 0.75 (Shallow Angle)///
    ///0.15 (Highest Angle) 0.75 (Shallow Angle)///
    ///0.15 (Highest Angle) 0.75 (Shallow Angle)///
    ///0.15 (Highest Angle) 0.75 (Shallow Angle)///
    ///0.15 (Highest Angle) 0.75 (Shallow Angle)///


    @JvmField var POS_63deg = 0.97;
    @JvmField var POS_55deg = 0.3;

    val open = SetPosition(servo, 0.1).requires(this)

    var targetTheta: Angle = 0.0.rad
        get() {
            return norm(
                8.0.deg *
                        ((servo.position - POS_55deg) / (POS_63deg - POS_55deg))
                        + 55.0.deg
            )
        }
        private set

    fun norm(angle: Angle): Angle {
        return min(max(atan2(sin(angle.inRad), cos(angle.inRad)).rad.inDeg, 55.0), 63.0).deg;
    }

    class SetTargetTheta(val angle: Angle) : SetPositions(
        servo to ((norm(angle) - 55.0.deg) / 8.0.deg * (POS_63deg - POS_55deg) + POS_55deg)
        //im not sure if this work
        //servo to (angle*0.09625 - 5.09375)
    )

    class AutoAim(
        private val dxy: Double,
        private val angleByDistance: (Double) -> Angle,  // get by running curve of best fit on collected data
    ) : Command() {
        override val isDone = true;

        init {
            setName("Auto Aim Theta")
        }
        
        override fun start() {
            SetTargetTheta(angleByDistance(dxy))();
            PanelsTelemetry.telemetry.addData("s", dxy)
            PanelsTelemetry.telemetry.addData("theta goal", angleByDistance(dxy))
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