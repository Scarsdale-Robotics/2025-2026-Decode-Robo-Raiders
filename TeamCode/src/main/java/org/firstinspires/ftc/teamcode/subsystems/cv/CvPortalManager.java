package org.firstinspires.ftc.teamcode.subsystems.cv;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;

import java.util.ArrayList;
import java.util.List;


class CvBallDetection {

    private final ColorBlobLocatorProcessor colorLocator;
    private List<ColorBlobLocatorProcessor.Blob> blobs = new ArrayList<>();

    public CvBallDetection(boolean isPurple) {
        ColorRange targetColor = isPurple ? ColorRange.ARTIFACT_PURPLE : ColorRange.ARTIFACT_GREEN;

        colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(targetColor)
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)
                .setBoxFitColor(0)
                .setCircleFitColor(Color.rgb(255, 255, 0))
                .setBlurSize(5)
                .setDilateSize(15)
                .setErodeSize(15)
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)
                .build();
    }

    public ColorBlobLocatorProcessor getProcessor() {
        return colorLocator;
    }

    public void update() {
        blobs = colorLocator.getBlobs();

        if (blobs != null && !blobs.isEmpty()) {
//            ColorBlobLocatorProcessor.Util.filterByCriteria(
//                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
//                    50, 20000, blobs);
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                    0.2, 1, blobs);
        }
    }

    public List<ColorBlobLocatorProcessor.Blob> getBlobs() {
        return blobs != null ? blobs : new ArrayList<>();
    }
}


class CvAprilTag {

    private final AprilTagProcessor aprilTagProcessor;

    private double x = 0, y = 0, h = 0;
    private boolean hasDetection = false;

    public CvAprilTag(double x1, double y1, double h1) {
        AprilTagLibrary tagLibrary = new AprilTagLibrary.Builder()
                .addTag(20, "BlueTarget",
                        6.5, new VectorF(-58.3727f, -55.6425f, 29.5f), DistanceUnit.INCH,
                        new Quaternion(0.2182149f, -0.2182149f, -0.6725937f, 0.6725937f, 0))
                .addTag(24, "RedTarget",
                        6.5, new VectorF(-58.3727f, 55.6425f, 29.5f), DistanceUnit.INCH,
                        new Quaternion(0.6725937f, -0.6725937f, -0.2182149f, 0.2182149f, 0))
                .build();

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(tagLibrary)
//                .setCameraPose(
//                        new Position(
//                                DistanceUnit.INCH, 4.25462244094, -4.50787402, 7.57202637795, 0
//                        ),
//                        new YawPitchRollAngles(
//                                AngleUnit.DEGREES,
//                                180 - Math.toDegrees(-0.0658537f) - Math.toDegrees(0.052),
//                                -105,
//                                180,
//                                0
//                        )
//                )
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .build();

        x = x1;
        y = y1;
        h = h1;
    }

    public AprilTagProcessor getProcessor() {
        return aprilTagProcessor;
    }

    public void update() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        if (detections == null || detections.isEmpty()) {
            hasDetection = false;
            return;
        }

        AprilTagDetection best = null;
        double bestRange = Double.MAX_VALUE;

        for (AprilTagDetection d : detections) {
            if (d.robotPose != null && d.ftcPose != null && d.ftcPose.range < bestRange) {
                best = d;
                bestRange = d.ftcPose.range;
            }
        }

        if (best == null) {
            hasDetection = false;
            return;
        }

        hasDetection = true;
        Pose3D pose = best.robotPose;

        Pose ftcStandard = PoseConverter.pose2DToPose(
                new Pose2D(
                        DistanceUnit.INCH,
                        pose.getPosition().toUnit(DistanceUnit.INCH).x,
                        pose.getPosition().toUnit(DistanceUnit.INCH).y,
                        AngleUnit.RADIANS,
                        pose.getOrientation().getYaw(AngleUnit.RADIANS)
                ),
                InvertedFTCCoordinates.INSTANCE
        );

        Pose cvtPose = ftcStandard.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        x = 72.0 - cvtPose.getX();
        y = 72.0 - cvtPose.getY();
        h = cvtPose.getHeading() % (2 * Math.PI) - Math.PI;
    }

    public boolean hasDetection() { return hasDetection; }
    public double getX1() { return x; }
    public double getY1() { return y; }
    public double getH1() { return h; }
    public void setCv(double x1, double y1, double h1) {
        x = x1;
        y = y1;
        h = h1;
    }
}


public class CvPortalManager {

    private final VisionPortal visionPortal;
    public final CvBallDetection ballDetection;
    public final CvAprilTag aprilTag;

    public CvPortalManager(double x1, double y1, double h1, boolean isPurple, HardwareMap hm) {
        ballDetection = new CvBallDetection(isPurple);
        aprilTag = new CvAprilTag(x1,y1,h1);
        aprilTag.setCv(x1,y1,h1);

        visionPortal = new VisionPortal.Builder()
                .setCamera(hm.get(WebcamName.class, "Cam"))
                .setCameraResolution(new android.util.Size(640, 480))
                .addProcessor(ballDetection.getProcessor())
                .addProcessor(aprilTag.getProcessor())
                .build();
    }

    public void update() {
        ballDetection.update();
        aprilTag.update();
    }

    public void close() {
        visionPortal.close();
    }

    public double getX2(){return aprilTag.getX1();}
    public double getY2(){return aprilTag.getY1();}
    public double getH2(){return aprilTag.getH1();}
    public boolean hasDetection1() { return aprilTag.hasDetection();}
    public void setCv1(double x1, double y1,double h1) {aprilTag.setCv(x1,y1,h1);}
    public List<ColorBlobLocatorProcessor.Blob> getBlobs1() {return ballDetection.getBlobs();}
}