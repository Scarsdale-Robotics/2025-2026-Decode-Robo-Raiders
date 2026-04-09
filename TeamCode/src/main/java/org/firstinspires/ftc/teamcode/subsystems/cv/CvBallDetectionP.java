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
    private ColorBlobLocatorProcessor colorLocatorPurp;
    private ColorBlobLocatorProcessor colorLocatorGreen;

    private VisionPortal visionPortal;
    private List<ColorBlobLocatorProcessor.Blob> blobs = new ArrayList<>(); // initialize to avoid null
    private List<ColorBlobLocatorProcessor.Blob> blobsG = new ArrayList<>(); // initialize to avoid null
    private List<ColorBlobLocatorProcessor.Blob> blobsP = new ArrayList<>(); // initialize to avoid null

    public CvBallDetectionP(HardwareMap hm) {

        this.colorLocatorPurp = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)
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

        this.colorLocatorGreen = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_GREEN)
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
                .addProcessor(colorLocatorPurp)
                .addProcessor(colorLocatorGreen)
                .setCameraResolution(new android.util.Size(640, 480))
                .setCamera(hm.get(WebcamName.class, "Cam"))
                .build();
    }

    public void updateDetections() {
        blobs.clear();

       blobsG = colorLocatorGreen.getBlobs();
       blobsP = colorLocatorPurp.getBlobs();
       if(blobsG != null && !blobsG.isEmpty()){
           ColorBlobLocatorProcessor.Util.filterByCriteria(
                   ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                   400, 50000, blobsG);
           ColorBlobLocatorProcessor.Util.filterByCriteria(
                   ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                   0.55, 1, blobsG);
           this.blobs.addAll(blobsG);
       }
       if(blobsP != null && !blobsP.isEmpty()){
           ColorBlobLocatorProcessor.Util.filterByCriteria(
                   ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                   400, 50000, blobsP);
           ColorBlobLocatorProcessor.Util.filterByCriteria(
                   ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                   0.55, 1, blobsP);
           this.blobs.addAll(blobsP);
       }


//        if (blobs != null && !blobs.isEmpty()) {
//            ColorBlobLocatorProcessor.Util.filterByCriteria(
//                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
//                    400, 50000, blobs);
//
//            ColorBlobLocatorProcessor.Util.filterByCriteria(
//                    ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
//                    0.55, 1, blobs);
//
////            ColorBlobLocatorProcessor.Util.filterByCriteria(
////                    ColorBlobLocatorProcessor.BlobCriteria.BY_DENSITY,0.5,1, blobs
////            );
//
//        }
    }

    public List<ColorBlobLocatorProcessor.Blob> getBlobs() {
        return blobs != null ? blobs : new ArrayList<>();
    }
}