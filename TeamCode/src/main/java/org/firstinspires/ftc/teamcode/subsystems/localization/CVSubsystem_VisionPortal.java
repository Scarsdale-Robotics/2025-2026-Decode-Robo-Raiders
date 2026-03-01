package org.firstinspires.ftc.teamcode.subsystems.localization;

import androidx.annotation.Nullable;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class CVSubsystem_VisionPortal {

    private final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTagProcessor;

    private double RCx1;
    private double RCy1;
    private double RCh;

    private AprilTagDetection lastDetection;

    private static final double CAM_X = -4.88119055118;
    private static final double CAM_Y = -4.38726968504;
    private static final double CAM_Z = 0.0; // tune

    public CVSubsystem_VisionPortal(double x1, double y1, double h, HardwareMap hm) {


        AprilTagLibrary tagLibrary = new AprilTagLibrary.Builder()
                .addTags(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .build();

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(tagLibrary)
                .setCameraPose(
                        new Position(
                                DistanceUnit.INCH,
                                CAM_X,
                                CAM_Y,
                                CAM_Z,
                                0
                        ), new YawPitchRollAngles(
                                AngleUnit.RADIANS,
                                Math.PI,
                                -Math.toRadians(15),
                                Math.PI,
                                0
                        )
                )
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .build();

        WebcamName webcamName = hm.get(WebcamName.class, "Cam");

        visionPortal = new VisionPortal.Builder()
                .setCamera(webcamName)
                .setCameraResolution(new android.util.Size(640, 480))
                .setLiveViewContainerId(
                        hm.appContext.getResources().getIdentifier(
                                "cameraMonitorViewId",
                                "id",
                                hm.appContext.getPackageName()
                        )
                )
                .addProcessor(aprilTagProcessor)
                .setAutoStartStreamOnBuild(true)
                .build();


        RCx1 = x1;
        RCy1 = y1;
        RCh = h;
    }



    public void closeCam() {
        visionPortal.close();
    }

    public void updateCV() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        if (detections == null || detections.isEmpty()) return;

        AprilTagDetection best = null;
        double bestRange = Double.MAX_VALUE;

        for (AprilTagDetection d : detections) {
            if (d.ftcPose != null && d.ftcPose.range < bestRange) {
                best = d;
                bestRange = d.ftcPose.range;
            }
        }

        if (best == null) return;
        if (best.robotPose == null) return;

        Pose3D pose = best.robotPose;

        RCx1 = pose.getPosition().toUnit(DistanceUnit.INCH).x;
        RCy1 = pose.getPosition().toUnit(DistanceUnit.INCH).y;
        RCh = pose.getOrientation().getYaw();

        lastDetection = best;
    }   
    

    public void setCv(double x1, double y1, double h) {
        RCx1 = x1;
        RCy1 = y1;
        RCh = h;
    }

    public double getRCx1() { return RCx1; }
    public double getRCy1() { return RCy1; }
    public double getRCh()  { return RCh; }
    

    public boolean hasDetection() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        return detections != null && !detections.isEmpty();
    }
}
