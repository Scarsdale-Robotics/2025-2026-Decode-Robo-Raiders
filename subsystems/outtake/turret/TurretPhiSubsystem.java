package org.firstinspires.ftc.teamcode.subsystems.outtake.turret;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;

// left-right
public class TurretPhiSubsystem implements Subsystem {
    public static TurretPhiSubsystem INSTANCE = new TurretPhiSubsystem();

    private final MotorEx motor_phi = new MotorEx("Motor Turret Phi");

    private ControlSystem controller_phi;

    public static double kSQU_phi = 0, kI_phi = 0, kD_phi = 0;  // TODO: TUNE
    public static int MIN_ENCODER_PHI = 0, MAX_ENCODER_PHI = 5000;  // TODO: TUNE

    private TurretPhiSubsystem() {
        controller_phi = ControlSystem.builder()
                .posSquid(kSQU_phi, kI_phi, kD_phi)
                .build();
        controller_phi.setGoal(new KineticState(0.0));
    }

    public Command setPhi(double phi) {
        return new InstantCommand(() -> controller_phi.setGoal(
                new KineticState(
                        ((phi / (2 * Math.PI) + 0.5) % 1) *
                                (MAX_ENCODER_PHI - MIN_ENCODER_PHI) + MIN_ENCODER_PHI
                )
        ));
    }

    // IMPORTANT: being at target phi does not mean no motor power,
    // could also be in process of overshooting, depending on tuned controller constants
    public boolean atTarget() {
        return controller_phi.isWithinTolerance(
                new KineticState(50)
        );
    }

    @Override
    public void periodic(){
        // move towards targets
        motor_phi.setPower(
                controller_phi.calculate(
                        new KineticState(
                                motor_phi.getCurrentPosition()
                        )
                )
        );
    }
}
