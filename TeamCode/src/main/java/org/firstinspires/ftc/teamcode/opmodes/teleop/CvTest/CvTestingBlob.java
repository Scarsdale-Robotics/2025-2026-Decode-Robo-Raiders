package org.firstinspires.ftc.teamcode.opmodes.teleop.CvTest;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.cv.CvBallDetectionP;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.List;

@TeleOp(name = "CVb67")
public class CvTestingBlob extends LinearOpMode {

    CvBallDetectionP Portal;
    List<ColorBlobLocatorProcessor.Blob> Blobs;
    double cd;

    @Override
    public void runOpMode() throws InterruptedException {
        this.Portal = new CvBallDetectionP(hardwareMap);
        waitForStart();


        while (opModeIsActive()) {
            double min = Double.MAX_VALUE;  // reset each loop iteration
            ColorBlobLocatorProcessor.Blob Minb = null;

            Portal.updateDetections();
            this.Blobs = Portal.getBlobs();

            if (Blobs != null && !Blobs.isEmpty()) {
                telemetry.addData("Blob count", Blobs.size());
                if(Blobs.isEmpty()){cd = -1.0;}
                for (ColorBlobLocatorProcessor.Blob b : Blobs) {
                    Circle circleFit = b.getCircle();

                    if (circleFit == null) continue;

                    double radius = circleFit.getRadius();

                    if (radius == 0) continue;
                     cd = ((120.0 * 391) / (radius * 2))*2;
                    double theta = Math.atan2(circleFit.getX() - 320, 391);

                    telemetry.addData("Blob @ X=" + (int)circleFit.getX() + " Circularity", b.getCircularity());
                    telemetry.addData("Blob @ X=" + (int)circleFit.getX() + " Radius", radius);
                    telemetry.addData("Blob @ X=" + (int)circleFit.getX() + " Y", circleFit.getY());
                    telemetry.addData("Blob @ X=" + (int)circleFit.getX() + " Distance", cd);
                    telemetry.addData("Blob @ X=" + (int)circleFit.getX() + " Angle", theta);

                    if (cd < min) {
                        min = cd;
                        Minb = b;
                    }
                }

            }

            telemetry.update();
        }
    }
}