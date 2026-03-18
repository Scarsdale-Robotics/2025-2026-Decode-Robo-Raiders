package org.firstinspires.ftc.teamcode.subsystems.outtake.turret;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.function.Supplier;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.units.Angle;
import dev.nextftc.hardware.controllable.RunToState;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.ServoGroup;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.powerable.SetPower;

@Configurable
public class TurretPhiJava implements Subsystem {
    public static final TurretPhiJava INSTANCE = new TurretPhiJava();
    private TurretPhiJava(){

    }
    private ServoEx servo1 = new ServoEx("servo_one");
    private ServoEx servo2 = new ServoEx("servo_two");
    private final ServoGroup servoGroup = new ServoGroup(
            servo1,
            servo2
    );
    public Command setTurretPhi = new SetPosition(servoGroup, 0.4).requires(this);
    @Override
    public void initialize() {
    }

    public Angle getTargetPhi() {
        double val = Math.PI * (motor.getCurrentPosition() - ENCODERS_FORWARD)
                / (ENCODERS_FORWARD - ENCODERS_BACKWARD);
        return Angle.rad(val);
    }

    public Angle norm(Angle angle) {
        double tolerance = Math.toRadians(10.0);
        double dzHalf = Math.toRadians(19.0);
        double a = angle.inRad();

        double LOWER = Math.toRadians(51.0) - 2 * Math.PI;
        double UPPER = Math.toRadians(51.0);

        while (a < LOWER - tolerance) {
            a += 2 * Math.PI;
        }
        while (a > UPPER + tolerance) {
            a -= 2 * Math.PI;
        }

        double clamped = Math.max(LOWER + dzHalf, Math.min(UPPER - dzHalf, a));
        return Angle.rad(clamped);
    }

    // ================= COMMANDS =================


    public class AutoAim extends Command {

        private final double dx, dy;
        private final Angle rh;
        private final Angle ofsTurret;
        private final double feedforward;

        public AutoAim(double dx, double dy, Angle rh,
                       Angle ofsTurret, double feedforward) {
            this.dx = dx;
            this.dy = dy;
            this.rh = rh;
            this.ofsTurret = ofsTurret;
            this.feedforward = feedforward;
            setName("Auto Aim Phi");
        }

        public AutoAim(double dx, double dy, Angle rh) {
            this(dx, dy, rh, Angle.rad(0), 0.0);
        }

        @Override
        public boolean isDone() {
            return true;
        }

        @Override
        public void start() {
            if (lastCommand != null) {
                CommandManager.cancelCommand(lastCommand);
            }

            double normAngle = Math.atan2(dy, dx) - rh.inRad();

            double upper = normAngle + 2 * Math.PI;
            double lower = normAngle - 2 * Math.PI;

            double target = getTargetPhi().inRad();

            double closest = normAngle;
            double closestDist = Math.abs(normAngle - target);

            if (Math.abs(upper - target) < closestDist) {
                closest = upper;
                closestDist = Math.abs(upper - target);
            }

            if (Math.abs(lower - target) < closestDist) {
                closest = lower;
            }

            SetTargetPhi cmd = new SetTargetPhi(Angle.rad(closest), ofsTurret);
            cmd.schedule(); // 🔥 instead of ()
            lastCommand = cmd;
        }
    }

    public class Manual extends Command {

        private final Supplier<Double> goalChange;

        public Manual(Supplier<Double> goalChange) {
            this.goalChange = goalChange;
        }

        @Override
        public boolean isDone() {
            return false;
        }

        @Override
        public void update() {
            new RunToState(
                    controller,
                    new KineticState(
                            controller.getGoal().getPosition()
                                    + goalChange.get() * thing
                    )
            ).schedule();
        }
    }

    // ================= LOOP =================

    @Override
    public void periodic() {
        if (!started) return;

        secondaryController.setGoal(controller.getGoal());

        double power = controller.calculate(motor.getState());

        double error = Math.abs(
                controller.getGoal().getPosition()
                        - controller.getLastMeasurement().getPosition()
        );

        if (error < 6.7) {
            power = 0.0;
        } else if (error < 111.111) {
            power = secondaryController.calculate(motor.getState());
            power += feedforwardCmd * feedforwardCoefficient;
        }

        new SetPower(motor, power)
                .setInterruptible(true)
                .schedule();

        PanelsTelemetry.telemetry.addData("phi enc", motor.getCurrentPosition());
        PanelsTelemetry.telemetry.addData("ref", controller.getReference());
        PanelsTelemetry.telemetry.addData("goal", controller.getGoal());
        PanelsTelemetry.telemetry.addData("turret phi", getTargetPhi());
        PanelsTelemetry.telemetry.addData("lm", controller.getLastMeasurement());
    }
}