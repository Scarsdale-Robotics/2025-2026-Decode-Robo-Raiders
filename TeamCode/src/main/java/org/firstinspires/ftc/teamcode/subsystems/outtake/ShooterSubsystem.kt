package org.firstinspires.ftc.teamcode.subsystems.outtake

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry.telemetry
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.control.ControlSystem
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.FeedbackType
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.nextftc.core.commands.utility.InstantCommand
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.controllable.RunToVelocity
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower
import org.firstinspires.ftc.teamcode.utils.SMO.SMOFilter


@Configurable
object ShooterSubsystem : Subsystem {
    private val motor = MotorEx("shooter").reversed();

    @JvmField var ffCoefficients = BasicFeedforwardParameters(0.03, 0.02, 0.01);
    @JvmField var squidCoefficients = PIDCoefficients(0.016, 0.0, 0.000)

//    @JvmField var MAX_VELOCITY = -1000.0;  // rotational velocity
//    @JvmField var NO_VELOCITY = 0.0;

//    private var targetShooterVelocity = NO_VELOCITY;
    private val controller: ControlSystem;

    init {
//        val velSMO = SMOFilter(FeedbackType.VELOCITY, Ls, Lv, La)
        controller = controlSystem {
//            velFilter { filter -> filter.custom(velSMO).build() }
//            basicFF(ffCoefficients)

            velSquID(squidCoefficients)
        }
    }

//    @JvmField var MAX_SPEED = 1423.0;

    @JvmField var on = RunToVelocity(controller, 1600.0).requires(this).named("FlywheelOn").setInterruptible(true);
    @JvmField var off = RunToVelocity(controller, 0.0).requires(this).named("FlywheelOff").setInterruptible(true);

    var lastPos = 0.0;
    var elapsedTime: ElapsedTime = ElapsedTime();
    override fun periodic() {
        val power = controller.calculate(
            motor.state * -1.0
        );
        SetPower(motor, power).setInterruptible(true)()

        val measuredVel = (motor.currentPosition - lastPos)/elapsedTime.time();
        lastPos = motor.currentPosition;
        elapsedTime.reset()

        telemetry.addData("power", power)

        telemetry.addData("vel measured", measuredVel)
        telemetry.addData("vel est", controller.lastMeasurement.velocity)
        telemetry.addData("vel ref", controller.reference.velocity)
        telemetry.addData("vel goal", controller.goal.velocity)

        telemetry.addData("pos measured", motor.currentPosition)
        telemetry.addData("pos est", controller.lastMeasurement.position)
        telemetry.addData("pos ref", controller.reference.position)



//        dashboardTelemetry.addData("lastMeasurement", controlSystem.lastMeasurement.velocity)
//        dashboardTelemetry.addData("ref", controlSystem.reference.velocity)
//        dashboardTelemetry.addData("goal", controlSystem.goal.velocity)
    }
}