package org.firstinspires.ftc.teamcode.opmodes.teleop.LocalTests;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.Circle;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;

import java.util.List;


/*    .setTargetColorRange(ColorRange.BLUE)     // use a predefined color match
 *
 *
 * - Focus the color locator by defining a RegionOfInterest (ROI) which you want to search.
 *     This can be the entire frame, or a sub-region defined using:
 *     1) standard image coordinates or 2) a normalized +/- 1.0 coordinate system.
 *     Use one form of the ImageRegion class to define the ROI.
 *
 * - Define which contours are included.
 *   You can get ALL the contours, ignore contours that are completely inside another contour.
 *
 *
 * - Turn the displays of contours ON or OFF.
 *     Turning these on helps debugging but takes up valuable CPU time.
 *
 *
 * - include any pre-processing of the image or mask before looking for Blobs.
 *     There are some extra processing you can include to improve the formation of blobs.
 *     Using these features requires an understanding of how they may effect the final
 *     blobs.  The "pixels" argument sets the NxN kernel size.
 *        .setBlurSize(int pixels)
 *        Blurring an image helps to provide a smooth color transition between objects,
 *        and smoother contours.  The higher the number, the more blurred the image becomes.
 *        Note:  Even "pixels" values will be incremented to satisfy the "odd number" requirement.
 *        Blurring too much may hide smaller features.  A size of 5 is good for a 320x240 image.
 *
 *     .setErodeSize(int pixels)
 *        Erosion removes floating pixels and thin lines so that only substantive objects remain.
 *        Erosion can grow holes inside regions, and also shrink objects.
 *        "pixels" in the range of 2-4 are suitable for low res images.
 *
 *     .setDilateSize(int pixels)
 *        Dilation makes objects and lines more visible by filling in small holes, and making
 *        filled shapes appear larger. Dilation is useful for joining broken parts of an
 *        object, such as when removing noise from an image.
 *        "pixels" in the range of 2-4 are suitable for low res images.
 *
 *        .setMorphOperationType(MorphOperationType morphOperationType)
 *        This defines the order in which the Erode/Dilate actions are performed.
 *        OPENING:    Will Erode and then Dilate which will make small noise blobs go away
 *        CLOSING:    Will Dilate and then Erode which will tend to fill in any small holes in blob edges.
 */

/*
 * The list of Blobs can be filtered to remove unwanted Blobs.
 *   Note:  All contours will be still displayed on the Stream Preview, but only those
 *          that satisfy the filter conditions will remain in the current list of
 *          "blobs".  Multiple filters may be used.
 *
 * To perform a filter
 *   ColorBlobLocatorProcessor.Util.filterByCriteria(criteria, minValue, maxValue, blobs);
 *
 * The following criteria are currently supported.
 *
 * ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA
 *   A Blob's area is the number of pixels contained within the Contour.  Filter out any
 *   that are too big or small. Start with a large range and then refine the range based
 *   on the likely size of the desired object in the viewfinder.
 *
 * ColorBlobLocatorProcessor.BlobCriteria.BY_DENSITY
 *   A blob's density is an indication of how "full" the contour is.
 *   If you put a rubber band around the contour you would get the "Convex Hull" of the
 *   contour. The density is the ratio of Contour-area to Convex Hull-area.
 *
 * ColorBlobLocatorProcessor.BlobCriteria.BY_ASPECT_RATIO
 *   A blob's Aspect ratio is the ratio of boxFit long side to short side.
 *   A perfect Square has an aspect ratio of 1.  All others are > 1
 *
 * ColorBlobLocatorProcessor.BlobCriteria.BY_ARC_LENGTH
 *   A blob's arc length is the perimeter of the blob.
 *   This can be used in conjunction with an area filter to detect oddly shaped blobs.
 *
 * ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY
 *   A blob's circularity is how circular it is based on the known area and arc length.
 *   A perfect circle has a circularity of 1.  All others are < 1
 */
@TeleOp(name = "Concept: Vision Color-Lo12cator (Circle)", group = "Concept")
public class ConceptVisionColorLocator_Circle extends LinearOpMode {
    @Override
    public void runOpMode() {

        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.ARTIFACT_PURPLE)   // Use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)
                .setRoi(ImageRegion.entireFrame())
                .setDrawContours(true)   // Show contours on the Stream Preview
                .setBoxFitColor(0)       // Disable the drawing of rectangles
                .setCircleFitColor(Color.rgb(255, 255, 0)) // Draw a circle
                .setBlurSize(5)          // Smooth the transitions between different colors in image

                // the following options have been added to fill in perimeter holes.
                .setDilateSize(15)       // Expand blobs to fill any divots on the edges
                .setErodeSize(15)        // Shrink blobs back to original size
                .setMorphOperationType(ColorBlobLocatorProcessor.MorphOperationType.CLOSING)

                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new android.util.Size(640, 480))
                .setCamera(hardwareMap.get(WebcamName.class, "Cam"))
                .build();

        telemetry.setMsTransmissionInterval(100);   // Speed up telemetry updates for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        // WARNING:  To view the stream preview on the Driver Station, this code runs in INIT mode.
        while (opModeIsActive() || opModeInInit()) {
            telemetry.addData("preview on/off", "... Camera Stream\n");

            // Read the current list
            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();

            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CONTOUR_AREA,
                    50, 20000, blobs);  // filter out very small blobs.

            ColorBlobLocatorProcessor.Util.filterByCriteria(
                    ColorBlobLocatorProcessor.BlobCriteria.BY_CIRCULARITY,
                    0.6, 1, blobs);



            telemetry.addLine("Circularity Radius Center");

            for (ColorBlobLocatorProcessor.Blob b : blobs) {

                Circle circleFit = b.getCircle();
                telemetry.addLine(String.format("%5.3f      %3d     (%3d,%3d)",
                           b.getCircularity(), (int) circleFit.getRadius(), (int) circleFit.getX(), (int) circleFit.getY()));
            }

            telemetry.update();
            sleep(100);
        }
    }
}
