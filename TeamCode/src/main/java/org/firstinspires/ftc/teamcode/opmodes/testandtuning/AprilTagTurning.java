package org.firstinspires.ftc.teamcode.opmodes.testandtuning;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.Arrays;
import java.util.List;

@TeleOp(name="AprilTagTurning", group="Robot")
public class AprilTagTurning extends LinearOpMode {
    DriveSubsystem driveSubsystem;
    Limelight3A limelight;
    @Override
    public void runOpMode() throws InterruptedException {
        driveSubsystem = new DriveSubsystem(this);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();
        while (opModeIsActive()){
            if (!Arrays.equals(getLimelightDegree(limelight), new double[]{-1, -1})){
                double xDegree = getLimelightDegree(limelight)[0];
                driveSubsystem.turnTo(driveSubsystem.getHeading() + xDegree);
            }

            telemetry.update();
        }
        limelight.stop();
    }

    public double[] getLimelightDegree(Limelight3A limelight) {
        LLResult result = limelight.getLatestResult();
        double xDegree = -1;
        double yDegree = -1;
        if (result != null) {
            Pose3D botpose = result.getBotpose();
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
            double parseLatency = result.getParseLatency();
            telemetry.addData("LL Latency", captureLatency + targetingLatency);
            telemetry.addData("Parse Latency", parseLatency);
            telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

            if (result.isValid()) {
                telemetry.addData("tx", result.getTx());
                telemetry.addData("txnc", result.getTxNC());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("tync", result.getTyNC());

                telemetry.addData("Botpose", botpose.toString());

                // Access fiducial results
                List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fr : fiducialResults) {
                    telemetry.addData("Fiducial", "ID: %d, Family: %s, X: %.2f, Y: %.2f", fr.getFiducialId(), fr.getFamily(),fr.getTargetXDegrees(), fr.getTargetYDegrees());
                }

                // Access color results
                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                for (LLResultTypes.ColorResult cr : colorResults) {
                    telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                }
            }
        } else {
            telemetry.addData("Limelight", "No data available");
        }

    return new double[] {xDegree, yDegree};
    }
}
