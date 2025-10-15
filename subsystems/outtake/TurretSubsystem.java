//package org.firstinspires.ftc.teamcode.subsystems.outtake;
//
//import static org.firstinspires.ftc.teamcode.utils.QuarticMaxNonnegRoot.maxNonNegativeRoot;
//
//import com.acmerobotics.dashboard.config.Config;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.subsystems.LocalizationSubsystem;
//
//import dev.nextftc.core.commands.Command;
//import dev.nextftc.core.commands.CommandManager;
//import dev.nextftc.core.commands.groups.ParallelGroup;
//import dev.nextftc.core.commands.utility.InstantCommand;
//import dev.nextftc.core.subsystems.SubsystemGroup;
//import dev.nextftc.ftc.ActiveOpMode;
//
//@Config
//public class TurretSubsystem extends SubsystemGroup {
//    public static final TurretSubsystem INSTANCE = new TurretSubsystem();
//
//    private TurretSubsystem() {
//        super(
//                TurretPhiSubsystem.INSTANCE,
//                TurretThetaSubsystem.INSTANCE
//        );
//    }
//
//    public Command setAim(double theta, double phi) {
//        return new ParallelGroup(
//                TurretThetaSubsystem.INSTANCE.setTheta(theta),
//                TurretPhiSubsystem.INSTANCE.setPhi(phi)
//        );
//    }
//
//
//    private final int TILE_SIZE = 24;
//    private final int FIELD_SIZE = 6 * TILE_SIZE;
//
//    public static boolean isRed = false;
//
//    private final double g = 386.08858267717;  // inches/sec^2
//    private final double v0 = 5;  // inches/sec  // TODO: TEST, tune, check if angle changes significantly, etc.
//
//    public Command autoAim() {
//        return new InstantCommand(() -> {
//            // TARGET INFO (T = Target)
//            double xT = isRed ? (FIELD_SIZE - TILE_SIZE / 2.0) : (TILE_SIZE / 2.0);
//            double yT = TILE_SIZE / 2.0;
//            double zT = 42.0;  // 38.75 is min, 53.75 is max
//
//            // TURRET INFO (U = Turret)
//            double xU = LocalizationSubsystem.INSTANCE.getXR() + 0;  // TODO: add turret offsets
//            double yU = LocalizationSubsystem.INSTANCE.getYR() + 0;
//            double zU = 5.0;
//
//            // ROBOT INFO (R = Robot)
//            double hR = LocalizationSubsystem.INSTANCE.getHR();
//
//            double vxU = LocalizationSubsystem.INSTANCE.getVXR();  // more representative of vxR, but close enough
//            double vyU = LocalizationSubsystem.INSTANCE.getVYR();
//            double vUmag = Math.hypot(vxU, vyU);
//
//            if (Math.abs(vxU) > 5 || Math.abs(vyU) > 5) {
//                ActiveOpMode.telemetry().addLine("(!) [ATA] Velocity spike (" + vxU + " in/s, " + vyU + " in/s)");
//            }
//
//            // Deltas
//            double dz = zT - zU;
//            double dy = yT - yU;
//            double dx = xT - xU;
////        double drMag = Math.sqrt(dx * dx + dy * dy + dz * dz);
//
//            double[] coeffs = {
//                    0.25 * g * g,
//                    0,
//                    vxU * vxU + vyU * vyU + g * dz - v0 * v0,
//                    -2 * (dx * vxU + dy * vyU),
//                    dx * dx + dy * dy + dz * dz
//            };
//            double t = maxNonNegativeRoot(coeffs);
//
//            if (Double.isNaN(t)) {
//                ActiveOpMode.telemetry().addLine("(!) [ATA] No positive time found");
//            }
//
//            // note notation opposite Stephen's doc, equal to subsystems, phi l/r, theta up/down
//            double theta_res = Math.asin((dz + 0.5 * g * t * t) / (v0 * t));
//            double phi_res = Math.atan2(dy - vyU * t, dx - vxU * t);
//
//            ActiveOpMode.telemetry().addData("[ATA] t", "%.3f s", t);
//            ActiveOpMode.telemetry().addData("[ATA] θ", "%.2f deg", Math.toDegrees(theta_res));
//            ActiveOpMode.telemetry().addData("[ATA] φ", "%.2f deg", Math.toDegrees(phi_res));
//
//            ActiveOpMode.telemetry().update();
//
//            CommandManager.INSTANCE.scheduleCommand(
//                    TurretSubsystem.INSTANCE.setAim(theta_res, phi_res)
//            );  // could also make a method that just returns setAim(theta_res, phi_res), but this probably has a more conventional style because I feel like it
//        });
//    }
//
//    /**
//     * Auto-Aim
//     * @param telemetry Deprecated
//     * @return Command for one auto-aim step
//     */
//    public Command autoAim(Telemetry telemetry) {
//        return autoAim();
//    }
//
//    public boolean atTarget() {
//        return
//                TurretThetaSubsystem.INSTANCE.atTarget() &&
//                TurretPhiSubsystem.INSTANCE.atTarget();
//    }
//}
