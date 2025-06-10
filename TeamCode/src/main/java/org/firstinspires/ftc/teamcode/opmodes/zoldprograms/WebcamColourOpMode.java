package org.firstinspires.ftc.teamcode.opmodes.zoldprograms;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorBlobLocatorProcessor;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.opencv.core.RotatedRect;

import java.util.List;

@TeleOp(name = "WebcamDetectColour", group = "zoldprograms")
public class WebcamColourOpMode  extends LinearOpMode
{
    @Override
    public void runOpMode()
    {
        LimelightSubsystem limelightSubsystem = new LimelightSubsystem(this);
        ColorBlobLocatorProcessor colorLocator = new ColorBlobLocatorProcessor.Builder()
                .setTargetColorRange(ColorRange.BLUE)         // use a predefined color match
                .setContourMode(ColorBlobLocatorProcessor.ContourMode.EXTERNAL_ONLY)    // exclude blobs inside blobs
                .setRoi(ImageRegion.asUnityCenterCoordinates(-1, 1, 1, -0.15))  // search central 1/4 of camera view
                .setDrawContours(true)                        // Show contours on the Stream Preview
                .setBlurSize(5)                               // Smooth the transitions between different colors in image
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .addProcessor(colorLocator)
                .setCameraResolution(new Size(320, 240))
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .build();

        telemetry.setMsTransmissionInterval(50);   // Speed up telemetry updates, Just use for debugging.
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        while (opModeIsActive() || opModeInInit()) {
            telemetry.addData("preview on/off", "... Camera Stream\n");
            List<ColorBlobLocatorProcessor.Blob> blobs = colorLocator.getBlobs();
            ColorBlobLocatorProcessor.Util.filterByArea(50, 20000, blobs);  // filter out very small blobs.

            telemetry.addLine(" Area Density Aspect  Center   degree");

            // Display the size (area) and center location for each Blob.
            for(ColorBlobLocatorProcessor.Blob b : blobs)
            {
                RotatedRect boxFit = b.getBoxFit();
                double degree = boxFit.center.x / 318 * 68.5 - (68.5 / 2);
                telemetry.addLine(String.format("%5d  %4.2f   %5.2f  (%3d,%3d)",
                        b.getContourArea(), b.getDensity(), b.getAspectRatio(), (int) boxFit.center.x, (int) boxFit.center.y));
                telemetry.addData("Webcam degree", degree);
                telemetry.addData("b angle", boxFit.angle);
            }

            double[] sampleRes = limelightSubsystem.getBlueSample();
            if (sampleRes != limelightSubsystem.ERROR_SAMPLE_RESULT) {
                telemetry.addData("LL degree", sampleRes[0]);
            } else {
                telemetry.addLine("LL degree not found");
            }
            telemetry.update();
            sleep(50);
        }
    }
}
