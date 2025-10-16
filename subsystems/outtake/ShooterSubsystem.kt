package org.firstinspires.ftc.teamcode.subsystems.outtake

import android.service.controls.Control
import com.acmerobotics.dashboard.config.Config
import dev.nextftc.control.ControlSystem
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.FeedbackType
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import org.firstinspires.ftc.teamcode.utils.SMO.SMOFilter
import java.time.Instant

@Config
object ShooterSubsystem : Subsystem {
    private val motor = MotorEx("shooter");

    @JvmField var Ls = 0.0;
    @JvmField var Lv = 0.0;
    @JvmField var La = 0.0;

    @JvmField var MAX_VELOCITY = 1.0;
    @JvmField var NO_VELOCITY = 0.0;

    private var targetShooterVelocity = NO_VELOCITY;
    private val controlSystem: ControlSystem;

    init {
        val velSMO = SMOFilter(FeedbackType.VELOCITY, Ls, Lv, La)
        controlSystem = controlSystem {
            velFilter { filter -> filter.custom(velSMO).build() }
            velSquID(Ls, Lv, La)
        }
    }

    class Run : InstantCommand({ targetShooterVelocity = MAX_VELOCITY; });
    class Stop : InstantCommand({ targetShooterVelocity = NO_VELOCITY; });

    override fun periodic() {
        motor.power = controlSystem.calculate(
            KineticState(0.0, targetShooterVelocity)  // todo: if issue (no move after bit), consider nonzero position here
        )
    }
}