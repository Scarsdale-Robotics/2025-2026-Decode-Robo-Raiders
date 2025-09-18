package org.firstinspires.ftc.teamcode.subsystems.outtake;

import com.acmerobotics.dashboard.config.Config;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

@Config
public class TurretSubsystem implements Subsystem {

    private final MotorEx motor_phi;
    private final ServoEx servo_theta;
    public static final TurretSubsystem INSTANCE = new TurretSubsystem();

    private ControlSystem controller_phi;

    // theta is around the local turret-forward field-parallel axis, 0 is facing field-parallel
    // phi is around field-normal axis (within field-parallel plane, 0 is forward)

    public static double kSQU_phi = 0, kI_phi = 0, kD_phi = 0;  // TODO: TUNE

    public static int MIN_ENCODER_PHI = 0, MAX_ENCODER_PHI = 5000;  // TODO: TUNE
    public static double MIN_SERVO_THETA = 0, MAX_SERVO_THETA = 1;  // TODO: TUNE


    private TurretSubsystem() {
        motor_phi = new MotorEx("Motor Turret Phi");
        controller_phi = ControlSystem.builder()
                .posSquid(kSQU_phi, kI_phi, kD_phi)
                .build();
        controller_phi.setGoal(new KineticState(0.0));

        servo_theta = new ServoEx("Servo Turret Theta");
        servo_theta.setPosition(0.0);
    }

    public void setGoal(double theta, double phi) {
        setTheta(theta); setPhi(phi);
    }

    public void setTheta(double theta) {
        servo_theta.setPosition(
                (theta / (2 * Math.PI) + 0.5) % 1 *
                (MAX_SERVO_THETA - MIN_SERVO_THETA) + MIN_SERVO_THETA
        );
    }

    public void setPhi(double phi) {
        controller_phi.setGoal(
                new KineticState(
                ((phi / (2 * Math.PI) + 0.5) % 1) *
                        (MAX_ENCODER_PHI - MIN_ENCODER_PHI) + MIN_ENCODER_PHI
                )
        );
    }

    public void onUpdate() {

        // move towards targets
        motor_phi.setPower(
                controller_phi.calculate(
                        new KineticState(
                                motor_phi.getCurrentPosition()
                        )
                )
        );

    }



    public void turretSetPower(double x, double y){
        // direct set motors
    }


}
