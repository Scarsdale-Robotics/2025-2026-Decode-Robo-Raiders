package org.firstinspires.ftc.teamcode.subsystems.localization;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import androidx.annotation.Nullable;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
///re written with Vision Portal supposedly the exact same thing
/// scrcpy --no-audio --record=file.mkv
public class CVSubsystem_VisionPortal {
    private final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTagProcessor;
    private final IMU imu;

    private boolean Cam;

    // Robot’s current pose (in inches)
    private double RCx1;
    private double RCy1;
    private double RCh; // heading in radians

    private double startingHeading = 0; // robot-centric offset

    private AprilTagDetection lastDetection;

    public HardwareMap hm1;

    public CVSubsystem_VisionPortal(double x1, double y1, double h, HardwareMap hm) {

        hm1 = hm;


        // Initialize IMU
        imu = hm1.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot( ///tune
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        AprilTagLibrary.Builder myAprilTagLibraryBuilder = new AprilTagLibrary.Builder();
        myAprilTagLibraryBuilder.addTags(AprilTagGameDatabase.getCurrentGameTagLibrary());
        myAprilTagLibraryBuilder.build();

        // Create AprilTag processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(myAprilTagLibraryBuilder.build()) ///check to make sure this works
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .build();

        // Create Vision Portal (using built-in or webcam)
        WebcamName webcamName = hm.get(WebcamName.class, "Cam");  ///has to be called Cam
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcamName)
                .enableLiveView(true)
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new android.util.Size(640, 480))
                .setCamera(BuiltinCameraDirection.BACK)// fallback if internal
                .setAutoStartStreamOnBuild(true)
                .setLiveViewContainerId(hm1.appContext.getResources().getIdentifier(
                        "cameraMonitorViewId", "id", hm1.appContext.getPackageName()
                ))
                .build();



        this.RCx1 = x1;
        this.RCy1 = y1;
        this.RCh = h;
        this.startingHeading = h;

        init();
    }

    public void init() {
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            visionPortal.resumeStreaming();
        }
        imu.resetYaw();
    }

    public void closeCam() {
        visionPortal.close();
        imu.resetYaw();
    }



    public void updateCV() {
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        if (detections == null || detections.isEmpty()) return;

        AprilTagDetection tag = detections.get(0);
        if (tag.robotPose == null) return;

        lastDetection = tag;

        Pose3D pose = tag.robotPose;
        RCx1 = pose.getPosition().x * 39.37;
        RCy1 = pose.getPosition().y * 39.37;

        double rawYaw = pose.getOrientation().getYaw(AngleUnit.RADIANS);

        RCh  = rawYaw - startingHeading;
        if (RCh > Math.PI) RCh -= 2*Math.PI;
        if (RCh < -Math.PI) RCh += 2*Math.PI;

//       if ((tag.id == 20 && side) || (tag.id == 24 && !side)) { ///Might work without this we dont need to allight to specific tag

//      }

    }

    public void setCv(double x1, double y1, double h) {
        this.RCx1 = x1;
        this.RCy1 = y1;
        this.RCh = h;
        this.startingHeading = h; // reset robot-centric reference
    }

    /// Getters ///
//    public boolean getSide() { return side; }
    public double getRCx1() { return RCx1; }
    public double getRCy1() { return RCy1; }
    public double getRCh() { return RCh; }

    @Nullable
    public AprilTagDetection getLastDetection() { return lastDetection; }

    public Boolean hasDetection(){
        List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
        return detections != null && !detections.isEmpty();
    }

}
