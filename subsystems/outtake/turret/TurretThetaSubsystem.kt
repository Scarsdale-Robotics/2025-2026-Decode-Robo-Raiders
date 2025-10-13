package org.firstinspires.ftc.teamcode.subsystems.outtake.turret

import com.acmerobotics.dashboard.config.Config
import dev.nextftc.core.commands.Command
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.core.units.Angle
import dev.nextftc.core.units.deg
import dev.nextftc.core.units.rad
import dev.nextftc.hardware.impl.ServoEx
import dev.nextftc.hardware.positionable.SetPosition
import java.util.function.Supplier
import kotlin.math.atan2
import kotlin.math.cos
import kotlin.math.sin

// up-down
@Config
object TurretThetaSubsystem : Subsystem {
    private val servo = ServoEx("turret_theta");

    @JvmField val POS_58deg = 0.0;
    @JvmField val POS_49deg = 0.5;  // todo: TUNE

    var targetTheta: Angle = 0.0.rad
        set(value) {
            val norm = atan2(sin(value.inRad), cos(value.inRad)).rad;
            field = norm;
            SetPosition(
                servo,
                (norm - 49.0.deg) / 9.0.deg *
                        (POS_58deg - POS_49deg) + POS_49deg
            );
        }

    class AutoAim(
        private val dxy: Supplier<Double>,
        private val angleByDistance: (Double) -> Angle,  // get by running curve of best fit on collected data
    ) : Command() {
        override val isDone = false;

        override fun update() {
            targetTheta = angleByDistance(dxy.get());
        }
    }

    class DriverCommand(

    ) // todo
}