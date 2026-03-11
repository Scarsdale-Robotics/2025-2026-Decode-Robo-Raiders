package org.firstinspires.ftc.teamcode.subsystems.localization;

import androidx.annotation.Nullable;

import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class CVSubsystem_VisionPortal {

    private final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTagProcessor;

    private double RCx1;
    private double RCy1;
    private double RCh;

    private AprilTagDetection lastDetection;

    private static final double CAM_X = -8; //-4.25462244094;
    private static final double CAM_Y = 5; //this might be postive beacuse this is the robots left
    private static final double CAM_Z = 7.6; // tune


//    double camXE;
//    double camYE;
    public CVSubsystem_VisionPortal(double x1, double y1, double h, HardwareMap hm) {


        AprilTagLibrary tagLibrary = new AprilTagLibrary.Builder()
                .addTag(20, "BlueTarget",
                        6.5, new VectorF(-58.3727f, -55.6425f, 29.5f), DistanceUnit.INCH,
                        new Quaternion(0.2182149f, -0.2182149f, -0.6725937f, 0.6725937f, 0))
                .addTag(24, "RedTarget",
                        6.5, new VectorF(-58.3727f, 55.6425f, 29.5f), DistanceUnit.INCH,
                        new Quaternion(0.6725937f, -0.6725937f, -0.2182149f, 0.2182149f, 0))
//                .addTags(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .build();
        //i hate kotlin, it sucks, and next ftc is so ahh
        //viir is king and we should all switch to ftclib
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(tagLibrary)
//                .setLensIntrinsics(1430,1457,480,620)
                .setCameraPose(
                        new Position(
                                DistanceUnit.INCH, 4.25462244094, -4.50787402, 7.57202637795, 0
                        ), new YawPitchRollAngles(
                                AngleUnit.DEGREES,
                                180,  // backwards
                                -105,  // 15 deg above horizontal
                                180,  // upside-down
                                0
                        )
                )
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.RADIANS)
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
            if (d.robotPose != null && d.ftcPose != null && d.ftcPose.range < bestRange) {
                best = d;
                bestRange = d.ftcPose.range;
            }
        }

        if (best == null) return;
//        if (best.ftcPose == null) return;


        Pose3D pose = best.robotPose;
        PanelsTelemetry.INSTANCE.getTelemetry().addData("pose", pose.toString());


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
        RCx1 = 72.0-cvtPose.getX();
        RCy1 = 72.0-cvtPose.getY();
        RCh = cvtPose.getHeading() % (2 * Math.PI) - Math.PI;

//        camXE = RCx1 - 72.0;
//        camYE = RCy1 -72.0;

//        RCx1 = pose.getPosition().toUnit(DistanceUnit.INCH).x + 72.0;
//        RCy1 = pose.getPosition().toUnit(DistanceUnit.INCH).y + 72.0;
//        RCh = pose.getOrientation().getYaw(AngleUnit.RADIANS);

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

    private double normalizeAngle(double angle) {
        while (angle > Math.PI)  angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    public boolean hasDetection() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        return detections != null && !detections.isEmpty();
    }

//    public double getCamXE() {
//        return camXE;
//    }
//
//    public double getCamYE() {
//        return camYE;
//    }
}