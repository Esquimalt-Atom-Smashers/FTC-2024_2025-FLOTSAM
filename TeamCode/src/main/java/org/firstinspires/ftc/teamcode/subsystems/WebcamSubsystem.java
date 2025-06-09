package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.List;

public class WebcamSubsystem extends SubsystemBase {
    private OpMode opMode;
    private HardwareMap hardwareMap;
    private ColorBlobLocatorProcessor colorLocator;
    private VisionPortal portal;
    private final double WEBCAM_FIELD_ANGLE = 66;
    private final double WEBCAM_POV_WIDTH = 320;
    public WebcamSubsystem(OpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
         colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, 0))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();

        portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size((int) WEBCAM_POV_WIDTH, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();
    }

    public double getBlueSample() {
        List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
        ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);

        if (blobs.isEmpty()) return Double.NaN;

        ColorBlobLocatorProcessor.Blob largestBlob = null;
        double largestArea = 0;

        for (ColorBlobLocatorProcessor.Blob blob : blobs) {
            double area = blob.getContourArea();
            if (area > largestArea) {
                largestArea = area;
                largestBlob = blob;
            }
        }

        if (largestBlob == null) return Double.NaN;

        RotatedRect boxFit = largestBlob.getBoxFit();
        double x = boxFit.center.x;
        return (x / WEBCAM_POV_WIDTH) * WEBCAM_FIELD_ANGLE - (WEBCAM_FIELD_ANGLE / 2);
    }

}
