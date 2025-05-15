package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

public class LimelightSubsystem extends SubsystemBase {
    OpMode opMode;
    Telemetry telemetry;

    Limelight3A limelight;
    public double METER_TO_INCH = 39.37008;
    public double[] ERROR_SAMPLE_RESULT = new double[] {10000, 10000};
    public double[] ERROR_POSE_3D = new double[] {10000, 10000, 10000};
    public Pose2d ERROR_POSE_2D = new Pose2d(10000 * METER_TO_INCH, 10000 * METER_TO_INCH, Math.toRadians(10000));

/*  limelight pos on bot:
    X 7.225 inch to the left
    Y 3.625 inch forward from middle
    Z 9 7/8 inch from ground
    
    Yaw 90 to the left
*/
    public LimelightSubsystem(OpMode opMode) {
        limelight = opMode.hardwareMap.get(Limelight3A.class, "limelight");
        this.telemetry = opMode.telemetry;
        this.telemetry.setMsTransmissionInterval(11);

        limelight.start();
    }

    public double[] getYellowSample() {
        limelight.pipelineSwitch(7);
        LLResult result = limelight.getLatestResult();
        double xDegree = 10000;
        double yDegree = 10000;

        if (result != null) {
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
//            telemetry.addData("LL Latency", captureLatency + targetingLatency);
            if (result.isValid()) {
                // Access color results
                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                for (LLResultTypes.ColorResult cr : colorResults) {
                    double xRes = cr.getTargetXDegrees();
                    double yRes = cr.getTargetYDegrees();
//                    telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                    if(Math.abs(xRes) < Math.abs(xDegree) && Math.abs(yRes) < Math.abs(yDegree)) {
                        xDegree = xRes;
                        yDegree = yRes;
                    }
                }
            }
        } else {
        }

        return new double[] {-xDegree, yDegree};
    }

    public double[] getRedSample() {
        limelight.pipelineSwitch(8);
        LLResult result = limelight.getLatestResult();
        double xDegree = 10000;
        double yDegree = 10000;

        if (result != null) {
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
//            telemetry.addData("LL Latency", captureLatency + targetingLatency);
            if (result.isValid()) {
                // Access color results
                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                for (LLResultTypes.ColorResult cr : colorResults) {
                    double xRes = cr.getTargetXDegrees();
                    double yRes = cr.getTargetYDegrees();
//                    telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                    if(Math.abs(xRes) < Math.abs(xDegree) && Math.abs(yRes) < Math.abs(yDegree)) {
                        xDegree = xRes;
                        yDegree = yRes;
                    }
                }
            }
        } else {
//            telemetry.addData("Limelight", "No data available");
        }

        return new double[] {-xDegree, yDegree};
    }

    public double[] getBlueSample() {
        limelight.pipelineSwitch(9);
        LLResult result = limelight.getLatestResult();
        double xDegree = 10000;
        double yDegree = 10000;

        if (result != null) {
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
//            telemetry.addData("LL Latency", captureLatency + targetingLatency);
            if (result.isValid()) {
                // Access color results
                List<LLResultTypes.ColorResult> colorResults = result.getColorResults();
                for (LLResultTypes.ColorResult cr : colorResults) {
                    double xRes = cr.getTargetXDegrees();
                    double yRes = cr.getTargetYDegrees();
//                    telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
                    if(Math.abs(xRes) < Math.abs(xDegree) && Math.abs(yRes) < Math.abs(yDegree)) {
                        xDegree = xRes;
                        yDegree = yRes;
                    }
                }
            }
        } else {
//            telemetry.addData("Limelight", "No data available");
        }

        return new double[] {-xDegree, yDegree};
    }

    public double[] getRobotPoseOnField() {
        limelight.pipelineSwitch(0);
        LLResult result = limelight.getLatestResult();
        double X = 10000;
        double Y = 10000;
        double Heading = 10000;
        if (result != null) {
            Pose3D botpose = result.getBotpose();
            double captureLatency = result.getCaptureLatency();
            double targetingLatency = result.getTargetingLatency();
//            double parseLatency = result.getParseLatency();
            telemetry.addData("LL Latency", captureLatency + targetingLatency);
//            telemetry.addData("Parse Latency", parseLatency);
//            telemetry.addData("PythonOutput", java.util.Arrays.toString(result.getPythonOutput()));

            if (result.isValid()) {
//                telemetry.addData("tx", result.getTx());
//                telemetry.addData("txnc", result.getTxNC());
//                telemetry.addData("ty", result.getTy());
//                telemetry.addData("tync", result.getTyNC());
                X = botpose.getPosition().x;
                Y = botpose.getPosition().y;
                Heading = botpose.getOrientation().getYaw();
//                telemetry.addData("Botpose", botpose.toString());
                }
        } else {
//            telemetry.addData("Limelight", "No data available");
        }
        return new double[] {X, Y, Heading};
    }

    public Pose2d lltoPose2d(double[] LLpose) {
        double LLX = LLpose[0] * METER_TO_INCH;
        double LLY = LLpose[1] * METER_TO_INCH;
        double LLA = Math.toRadians(LLpose[2]);
        return new Pose2d(LLX, LLY, LLA);
    }

    public Pose2d getLLCoorInAutoBlocking() {
        Pose2d correctPose = ERROR_POSE_2D;
        while (correctPose.equals(ERROR_POSE_2D)) {
            double[] result = getRobotPoseOnField();
            correctPose = lltoPose2d(result);
        }
        return correctPose;
    }

    public void limeLightStop() {
        limelight.stop();
    }
}
