package org.firstinspires.ftc.teamcode.opmodes.teleop.CvTest;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.cv.CvPortalManager;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.opencv.core.Mat;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.sql.Blob;
import java.util.List;

public class TwoPTest extends LinearOpMode {
    private static final Logger log = LoggerFactory.getLogger(TwoPTest.class);
    CvPortalManager Portal;
    Double x;
    Double y;
    Double h;
    List<ColorBlobLocatorProcessor.Blob> Blobs;
    final double FOV = 391.0;

    ColorBlobLocatorProcessor.Blob close;
    Blob big;
    @Override
    public void runOpMode() throws InterruptedException {
        this.Portal = new CvPortalManager(72,72,-Math.PI, true, hardwareMap);
        x = Portal.getX2();
        y = Portal.getY2();
        h = Portal.getH2();

        waitForStart();
        while(opModeIsActive()){
            x = Portal.getX2();
            y = Portal.getY2();
            h = Portal.getH2();

            this.Blobs = Portal.getBlobs1();
            double min = Double.MAX_VALUE;
            double max = Double.MIN_VALUE;

            if (Blobs != null && !Blobs.isEmpty()) {
                telemetry.addData("#Blobs", Blobs.size());
                for (ColorBlobLocatorProcessor.Blob b : Blobs) {
                    Circle circleFit = b.getCircle();
                    if (circleFit == null) continue;
                    double radius = circleFit.getRadius();
                    if (radius == 0) continue;
                    double cd = (120.0 * 391) / (radius * 2);
                    double theta = Math.atan2(circleFit.getX() - 320, 391);
                    telemetry.addData("Blob @ X=" + (int)circleFit.getX() + " Circularity", b.getCircularity());
                    telemetry.addData("Blob @ X=" + (int)circleFit.getX() + " Radius", radius);
                    telemetry.addData("Blob @ X=" + (int)circleFit.getX() + " Y", circleFit.getY());
                    telemetry.addData("Blob @ X=" + (int)circleFit.getX() + " Distance", cd);
                    telemetry.addData("Blob @ X=" + (int)circleFit.getX() + " Angle", theta);

                    if (cd < min) {
                        min = cd;
                        close = b;
                    }
                }
            }

            telemetry.addData("x: ", x);
            telemetry.addData("y: ", y);
            telemetry.addData("h: ", h);

            Portal.update();
            telemetry.update();
        }

    }
}
