package org.firstinspires.ftc.teamcode.subsystems.outtake

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry.telemetry
import com.qualcomm.robotcore.util.ElapsedTime
import dev.nextftc.control.ControlSystem
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.FeedbackType
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.nextftc.core.commands.Command
import dev.nextftc.core.commands.groups.ParallelGroup
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.core.units.Angle
import dev.nextftc.hardware.controllable.RunToState
import dev.nextftc.hardware.controllable.RunToVelocity
import dev.nextftc.hardware.impl.MotorEx
import dev.nextftc.hardware.powerable.SetPower
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem.targetTheta
import org.firstinspires.ftc.teamcode.utils.SMO.BasicSMOParameters
import org.firstinspires.ftc.teamcode.utils.SMO.SMOFilter
import java.util.function.Supplier


@Configurable
object ShooterSubsystem : Subsystem {
//    private val motor1 = MotorEx("shooter1");
    private val motor2 = MotorEx("shooter2").reversed();

    @JvmField var ffCoefficients = BasicFeedforwardParameters(0.0, 0.0, 0.75);
    @JvmField var pidCoefficients = PIDCoefficients(0.016, 0.0, 0.0)
    @JvmField var shootingCoefficients = PIDCoefficients(999999.9, 0.0, 0.0)

    private val controller: ControlSystem;
    private val shootingController: ControlSystem;

    init {
        controller = controlSystem {
            basicFF(ffCoefficients)
            velPid(pidCoefficients)
        }

        shootingController = controlSystem {
            basicFF(ffCoefficients)
            velPid(shootingCoefficients)
        }
    }

    fun setMotorPowers(power: Double) {
//        motor1.power = power;
        motor2.power = power;
    }

    class On(speed: Double) : RunToState(controller, KineticState(velocity=speed));
    var off = RunToVelocity(controller, 0.0).requires(this).named("FlywheelOff").setInterruptible(true);

    class Manual(
        private val shooterPower: Supplier<Double>
    ) : Command() {
        override val isDone = false;

        override fun update() {
            setMotorPowers(shooterPower.get())
        }
    }

    class AutoAim(
        private val dxy: Double,
        private val powerByDistance: (Double) -> Double,  // get by running curve of best fit on collected data
    ) : Command() {
        override val isDone = true;

        init {
            requires(ShooterSubsystem)
        }

        override fun start() {
            controller.goal = KineticState(velocity=powerByDistance(dxy));
        }
    }

    var isShooting = false;
    var lastPos = 0.0;
    var elapsedTime: ElapsedTime = ElapsedTime();
    override fun periodic() {
        var power = controller.calculate(
            motor2.state.times(-1.0)
        ).coerceIn(0.0, 1.0);

        if (isShooting) {
            power = shootingController.calculate(
                motor2.state.times(-1.0)
            ).coerceIn(0.0, 1.0)
        }

        setMotorPowers(power);

        val measuredVel = (motor2.currentPosition - lastPos)/elapsedTime.time();
        lastPos = motor2.currentPosition;
        elapsedTime.reset()

        telemetry.addData("power", power)

        telemetry.addData("vel measured", measuredVel)
        telemetry.addData("vel est", controller.lastMeasurement.velocity)
        telemetry.addData("vel ref", controller.reference.velocity)
        telemetry.addData("vel goal", controller.goal.velocity)

//        telemetry.addData("pos measured 1", motor1.currentPosition)
        telemetry.addData("pos measured 2", motor2.currentPosition)
        telemetry.addData("pos est", -controller.lastMeasurement.position)
        telemetry.addData("pos ref", controller.reference.position)
    }
}