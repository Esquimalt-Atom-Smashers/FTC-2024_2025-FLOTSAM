package org.firstinspires.ftc.teamcode.opmodes.testandtuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;

@TeleOp(name="SampleAiming", group="Robot")
public class SampleAiming extends LinearOpMode {
    DriveSubsystem driveSubsystem;
    LimelightSubsystem limelightSubsystem;
    @Override
    public void runOpMode() throws InterruptedException {
        driveSubsystem = new DriveSubsystem(this);
        limelightSubsystem = new LimelightSubsystem(this);

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
//            double[] redSampleLocation =  limelightSubsystem.getRedSample();
//            double redSampleHeading = redSampleLocation[1];
//            if (redSampleLocation == limelightSubsystem.ERROR_SAMPLE_RESULT) {
//                telemetry.addData("red sample heading:", "not found");
//            } else {
//                telemetry.addData("red sample heading:", redSampleHeading);
//            }
//
//            double[] yellowSampleLocation =  limelightSubsystem.getYellowSample();
//            double yellowSampleHeading = yellowSampleLocation[1];
//            if (yellowSampleLocation == limelightSubsystem.ERROR_SAMPLE_RESULT) {
//                telemetry.addData("red sample heading:", "not found");
//            } else {
//                telemetry.addData("red sample heading:", yellowSampleHeading);
//            }
            telemetry.update();

//            if (gamepad1.y) {
//                driveSubsystem.turnTo(yellowSampleHeading);
//            } else if (gamepad1.b) {
//                driveSubsystem.turnTo(redSampleHeading);
//            } else
            if (gamepad1.x && blueSampleHeading != limelightSubsystem.ERROR_SAMPLE_RESULT[0]) {
                driveSubsystem.turnTo(blueSampleHeading);
            } else {
                driveSubsystem.stopAll();
            }
        }
        limelightSubsystem.limeLightStop();
    }
}
