package org.firstinspires.ftc.teamcode.opmodes.Old;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.cv.CvPortalManager;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.List;

public class ahhh extends LinearOpMode {

    CvPortalManager portal;

    Double x;
    Double y;
    Double h;

    List<ColorBlobLocatorProcessor.Blob> blobs;

    // Tune this by holding the ball at a known distance and adjusting until Z matches
    final double FOV = 640.0;

    // Real diameter of the FTC sample ball in mm
    final double BALL_DIAMETER_MM = 120.0;

    // Camera resolution center (adjust if not 640x480)
    final double CENTER_X = 320.0;
    final double CENTER_Y = 240.0;

    ColorBlobLocatorProcessor.Blob closestBlob;

    @Override
    public void runOpMode() throws InterruptedException {
        portal = new CvPortalManager(72, 72, -Math.PI,  hardwareMap);

        x = portal.getX2();
        y = portal.getY2();
        h = portal.getH2();

        waitForStart();

        while (opModeIsActive()) {
            x = portal.getX2();
            y = portal.getY2();
            h = portal.getH2();

            blobs = portal.getBlobs1();

            double minDepth = Double.MAX_VALUE;
            closestBlob = null;

            if (blobs != null && !blobs.isEmpty()) {
                telemetry.addData("# Blobs", blobs.size());

                for (ColorBlobLocatorProcessor.Blob b : blobs) {
                    Circle circleFit = b.getCircle();
                    if (circleFit == null) continue;

                    double radius = circleFit.getRadius();
                    if (radius == 0) continue;

                    double diameterPixels = radius * 2;

                    // Solve for Z (depth) using pinhole camera model:
                    // px = FOV * realSize / Z  =>  Z = FOV * realSize / px
                    double Z = (FOV * BALL_DIAMETER_MM) / diameterPixels;

                    // Pixel offsets from image center
                    double px = circleFit.getX() - CENTER_X;
                    double py = circleFit.getY() - CENTER_Y;

                    // Back-solve for real-world X and Y:
                    // px = FOV * X / Z  =>  X = px * Z / FOV
                    double X = (px * Z) / FOV;
                    double Y = (py * Z) / FOV;

                    int bx = (int) circleFit.getX();
                    telemetry.addData("Blob @ px=" + bx + " | Z (mm)", String.format("%.1f", Z));
                    telemetry.addData("Blob @ px=" + bx + " | X (mm)", String.format("%.1f", X));
                    telemetry.addData("Blob @ px=" + bx + " | Y (mm)", String.format("%.1f", Y));
                    telemetry.addData("Blob @ px=" + bx + " | Radius (px)", String.format("%.1f", radius));
                    telemetry.addData("Blob @ px=" + bx + " | Circularity", String.format("%.2f", b.getCircularity()));

                    if (Z < minDepth) {
                        minDepth = Z;
                        closestBlob = b;
                    }
                }

                if (closestBlob != null) {
                    Circle c = closestBlob.getCircle();
                    double r = c.getRadius();
                    double Z = (FOV * BALL_DIAMETER_MM) / (r * 2);
                    double X = ((c.getX() - CENTER_X) * Z) / FOV;
                    double Y = ((c.getY() - CENTER_Y) * Z) / FOV;
                    telemetry.addLine("--- Closest Ball ---");
                    telemetry.addData("Z (mm)", String.format("%.1f", Z));
                    telemetry.addData("X (mm)", String.format("%.1f", X));
                    telemetry.addData("Y (mm)", String.format("%.1f", Y));
                }
            } else {
                telemetry.addLine("No blobs detected");
            }

            portal.update();
            telemetry.update();
        }
    }
}