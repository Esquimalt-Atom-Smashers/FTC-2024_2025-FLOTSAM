package org.firstinspires.ftc.teamcode.opmodes.testandtuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WebcamSubsystem;

@TeleOp(name="SampleAiming", group="Robot")
public class SampleAiming extends LinearOpMode {
    DriveSubsystem driveSubsystem;
    LimelightSubsystem limelightSubsystem;
    WebcamSubsystem webcamSubsystem;
    @Override
    public void runOpMode() throws InterruptedException {
        driveSubsystem = new DriveSubsystem(this);
        limelightSubsystem = new LimelightSubsystem(this);
        webcamSubsystem = new WebcamSubsystem(this);

        waitForStart();
        while (opModeIsActive()){
            double[] blueSampleLocation =  limelightSubsystem.getBlueSample();
            double blueSampleHeading = blueSampleLocation[0];
            if (blueSampleLocation == limelightSubsystem.ERROR_SAMPLE_RESULT) {
                telemetry.addData("blue sample heading:", "not found");
            } else {
                telemetry.addData("blue sample heading:", blueSampleHeading);
            }
            telemetry.addData("is error", blueSampleHeading != limelightSubsystem.ERROR_SAMPLE_RESULT[0]);

            double[] blueSample = webcamSubsystem.getBlueSample();
            double webcamHeading = blueSample[0];
            double blueSampleDistance = blueSample[1];
            telemetry.addData("webcam heading", webcamHeading);
            telemetry.addData("sample distance", blueSampleDistance);
            telemetry.update();

            if (gamepad1.x && blueSampleHeading != limelightSubsystem.ERROR_SAMPLE_RESULT[0]) {
                driveSubsystem.turnTo(blueSampleHeading);
            } else if (gamepad1.y && Double.isNaN(webcamHeading)) {
                driveSubsystem.turnTo(webcamHeading);
            }
            else {
                driveSubsystem.stopAll();
            }
        }
        limelightSubsystem.limeLightStop();
    }
}
