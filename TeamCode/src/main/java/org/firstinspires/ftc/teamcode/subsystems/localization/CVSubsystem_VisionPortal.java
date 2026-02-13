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
    private final IMU imu;

    private double RCx1;
    private double RCy1;
    private double RCh;

    private AprilTagDetection lastDetection;

    private static final double CAM_X = -4.88119055118;
    private static final double CAM_Y = -4.38726968504;
    private static final double CAM_Z = 0.0;

    public CVSubsystem_VisionPortal(double x1, double y1, double h, HardwareMap hm) {

        imu = hm.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        AprilTagLibrary tagLibrary = new AprilTagLibrary.Builder()
                .addTags(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .build();

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(tagLibrary)

                // (BROKEN)
                /*
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                */

                // FIX
                .setCameraPose(
                        new Position(
                                DistanceUnit.INCH,
                                CAM_X,
                                CAM_Y,
                                CAM_Z,
                                0
                        ), new YawPitchRollAngles(
                                AngleUnit.RADIANS,0,0,0,0
                        )
                )
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .build();

        WebcamName webcamName = hm.get(WebcamName.class, "Cam");

        visionPortal = new VisionPortal.Builder()

                // YOUR ORIGINAL CODE (BROKEN)
                /*
                .setCamera(webcamName)
                .setCamera(BuiltinCameraDirection.BACK)
                .setCameraResolution(new android.util.Size(640, 480))
                .setAutoStartStreamOnBuild(true)
                .setLiveViewContainerId(0)
                */

                // FIX
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

    public void init() {
        imu.resetYaw();
    }

    public void closeCam() {
        visionPortal.close();
    }

    public void updateCV() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        if (detections == null || detections.isEmpty()) return;

        AprilTagDetection tag = detections.get(0);
        if (tag.robotPose == null) return;

        Pose3D pose = tag.robotPose;

        // YOUR ORIGINAL CODE (BROKEN)
        /*
        double camX = pose.getPosition().toUnit(DistanceUnit.INCH).x;
        double camY = pose.getPosition().toUnit(DistanceUnit.INCH).y;

        double camHeading = pose.getOrientation().getYaw(AngleUnit.RADIANS);

        double offsetX = CAM_X * Math.cos(camHeading) - CAM_Y * Math.sin(camHeading);
        double offsetY = CAM_X * Math.sin(camHeading) + CAM_Y * Math.cos(camHeading);

        RCx1 = camX - offsetX;
        RCy1 = camY - offsetY;

        RCh = camHeading - startingHeading;
        */

        // FIX
        RCx1 = pose.getPosition().toUnit(DistanceUnit.INCH).x;
        RCy1 = pose.getPosition().toUnit(DistanceUnit.INCH).y;
        RCh = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        lastDetection = tag;
    }

    public void setCv(double x1, double y1, double h) {
        RCx1 = x1;
        RCy1 = y1;
        RCh = h;
    }

    public double getRCx1() { return RCx1; }
    public double getRCy1() { return RCy1; }
    public double getRCh()  { return RCh; }

    @Nullable
    public AprilTagDetection getLastDetection() {
        return lastDetection;
    }

    public boolean hasDetection() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        return detections != null && !detections.isEmpty();
    }
}
