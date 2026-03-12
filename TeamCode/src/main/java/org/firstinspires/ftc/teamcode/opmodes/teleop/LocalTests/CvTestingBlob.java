package org.firstinspires.ftc.teamcode.opmodes.teleop.LocalTests;

import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.TelemetryPluginConfig;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.subsystems.cv.CvBallDetectionP;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;

import java.util.List;

import kotlin.Unit;

public class CvTestingBlob extends LinearOpMode {

    CvBallDetectionP Portal;
    List<ColorBlobLocatorProcessor.Blob> Blobs;

    private TelemetryManager panelsManager;

    @Override
    public void runOpMode() throws InterruptedException {
        this.Portal = new CvBallDetectionP(true, hardwareMap);

        panelsManager = new TelemetryManager(
                () -> new TelemetryPluginConfig(), //defualt config
                (List<String> list) -> Unit.INSTANCE,
                (Long interval) -> Unit.INSTANCE
        );

        waitForStart();
        while(opModeIsActive()){
            Portal.updateDetections();
            this.Blobs = Portal.getBlobs();

            for (ColorBlobLocatorProcessor.Blob b : Blobs) {
                Circle circleFit = b.getCircle();
                panelsManager.addData("Circularity: ", b.getCircularity());
                panelsManager.addData("Radius: ", circleFit.getRadius());
                panelsManager.addData("X: ", circleFit.getX());
                panelsManager.addData("Y: ", circleFit.getY());
                double cd = (120.0*391)/ circleFit.getRadius()*2;
                panelsManager.addData("D: ", cd);
                double theta = Math.atan2(circleFit.getX()-320,391);
                panelsManager.addData("A: ", theta);
            }

        }

    }
}
