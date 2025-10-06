package org.firstinspires.ftc.teamcode.subsystems.outtake;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretPhiSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.outtake.turret.TurretThetaSubsystem;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.core.subsystems.SubsystemGroup;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.impl.ServoEx;

@Config
public class TurretSubsystem extends SubsystemGroup {
    public static final TurretSubsystem INSTANCE = new TurretSubsystem();

    private TurretSubsystem() {
        super(
                TurretPhiSubsystem.INSTANCE,
                TurretThetaSubsystem.INSTANCE
        );
    }

    public Command setAim(double theta, double phi) {
        return new ParallelGroup(
                TurretThetaSubsystem.INSTANCE.setTheta(theta),
                TurretPhiSubsystem.INSTANCE.setPhi(phi)
        );
    }

    public boolean atTarget() {
        return
                TurretThetaSubsystem.INSTANCE.atTarget() &&
                TurretPhiSubsystem.INSTANCE.atTarget();
    }
}
