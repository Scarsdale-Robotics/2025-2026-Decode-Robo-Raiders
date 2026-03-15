package org.firstinspires.ftc.teamcode.opmodes.teleop.LocalTests;

import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.TelemetryPluginConfig;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.cv.CvBallDetectionP;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.List;

import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedback.PIDController;
import kotlin.Unit;

@TeleOp(name = "CVb3")
public class CvTestingBlob extends LinearOpMode {

    CvBallDetectionP Portal;
    List<ColorBlobLocatorProcessor.Blob> Blobs;

    private TelemetryManager panelsManager;

    @Override
    public void runOpMode() throws InterruptedException {
        this.Portal = new CvBallDetectionP(true, hardwareMap);

        panelsManager = new TelemetryManager(
                () -> new TelemetryPluginConfig(),
                (List<String> list) -> Unit.INSTANCE,
                (Long interval) -> Unit.INSTANCE
        );

        waitForStart();

        PIDCoefficients coefficients = new PIDCoefficients(0.012, 0, 0.00245);
        PIDController drive = new PIDController(coefficients);

        while (opModeIsActive()) {
            double min = Double.MAX_VALUE;  // reset each loop iteration
            ColorBlobLocatorProcessor.Blob Minb = null;

            Portal.updateDetections();
            this.Blobs = Portal.getBlobs();

            if (Blobs != null && !Blobs.isEmpty()) {
                for (ColorBlobLocatorProcessor.Blob b : Blobs) {
                    Circle circleFit = b.getCircle();

                    if (circleFit == null) continue;

                    double radius = circleFit.getRadius();

                    if (radius == 0) continue;

                    panelsManager.addData("Circularity: ", b.getCircularity());
                    panelsManager.addData("Radius: ", radius);
                    panelsManager.addData("X: ", circleFit.getX());
                    panelsManager.addData("Y: ", circleFit.getY());

                    double cd = (120.0 * 391) / (radius * 2);  // fixed operator precedence
                    panelsManager.addData("D: ", cd);

                    double theta = Math.atan2(circleFit.getX() - 320, 391);
                    panelsManager.addData("A: ", theta);

                    if (cd < min) {
                        min = cd;
                        Minb = b;
                    }
                }
            }

            panelsManager.update();
        }
    }
}