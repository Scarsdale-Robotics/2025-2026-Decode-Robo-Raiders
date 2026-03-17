package org.firstinspires.ftc.teamcode.subsystems.cv;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.ArrayList;
import java.util.List;

public class CvBallDetectionP {
    private ColorBlobLocatorProcessor colorLocator;
    private VisionPortal visionPortal;
    private List<ColorBlobLocatorProcessor.Blob> blobs = new ArrayList<>(); // initialize to avoid null

    public CvBallDetectionP(Boolean isPurple, HardwareMap hm) {
        ColorRange targetColor = isPurple ? ColorRange.ARTIFACT_PURPLE : ColorRange.ARTIFACT_GREEN;

        this.colorLocator = new ColorBlobLocatorProcessor.Builder()
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

        this.visionPortal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new android.util.Size(640, 480))
                .setCamera(hm.get(WebcamName.class, "Cam"))
                .build();
    }

    public void updateDetections() {
        this.blobs = colorLocator.getBlobs();

        if (blobs != null && !blobs.isEmpty()) {
            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                    75, 50000, blobs);

            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                    0.55, 1, blobs);
        }
    }

    public List<ColorBlobLocatorProcessor.Blob> getBlobs() {
        return blobs != null ? blobs : new ArrayList<>();
    }
}