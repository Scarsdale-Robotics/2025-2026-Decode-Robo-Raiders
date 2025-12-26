package org.firstinspires.ftc.teamcode.subsystems;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class BallDetection extends OpenCvPipeline {

    ///hsv -> erode/dilate (open) -> GaussianBlur -> HoughCircles (testing)
    private Mat hsv = new Mat();
    private Mat mask = new Mat();
    private Mat blurred = new Mat();

    private final Mat kernel = Imgproc.getStructuringElement(
            Imgproc.MORPH_ELLIPSE, new Size(5, 5)
    );

    public boolean ballFound = false;
    public Point ballCenter = null;
    public double ballRadius = 0;

    @Override
    public Mat processFrame(Mat input) {
        ballFound = false;
        ballCenter = null;
        ballRadius = 0;

        Imgproc.cvtColor(input, hsv, 41); //hsv

        Mat redMask1 = new Mat();
        Mat redMask2 = new Mat();

        Core.inRange(
                hsv,
                new Scalar(0, 120, 70),
                new Scalar(10, 255, 255),
                redMask1
        );

        Core.inRange(
                hsv,
                new Scalar(170, 120, 70),
                new Scalar(180, 255, 255),
                redMask2
        );

        Core.add(redMask1, redMask2, mask);


        Imgproc.erode(mask, mask, kernel);
        Imgproc.dilate(mask, mask, kernel);


        Imgproc.GaussianBlur(
                mask,
                blurred,
                new Size(9, 9),
                2,
                2
        );


        Mat circles = new Mat();

        Imgproc.HoughCircles(
                blurred,
                circles,
                Imgproc.HOUGH_GRADIENT,
                1.2,            // dp
                50,             // dist between circles
                100,            // (Canny high threshold)
                20,             // (vote threshold)
                15,             // min radius (TUNE THIS)
                80              // max radius (TUNE THIS)
        );


        if (circles.cols() > 0) {

            double[] data = circles.get(0, 0);

            if (data != null) {
                ballFound = true;

                int cx = (int) Math.round(data[0]);
                int cy = (int) Math.round(data[1]);
                int radius = (int) Math.round(data[2]);

                ballCenter = new Point(cx, cy);
                ballRadius = radius;

                Imgproc.circle(
                        input,
                        ballCenter,
                        radius,
                        new Scalar(0, 255, 0),
                        2
                );

                Imgproc.circle(
                        input,
                        ballCenter,
                        3,
                        new Scalar(255, 0, 0),
                        -1
                );
            }
        }

        return input;
    }
}

