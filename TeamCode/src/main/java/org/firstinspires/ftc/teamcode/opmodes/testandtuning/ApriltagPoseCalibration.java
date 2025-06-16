package org.firstinspires.ftc.teamcode.opmodes.testandtuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
@Disabled
@TeleOp
public class ApriltagPoseCalibration extends OpMode {
    MecanumDrive mecanumDrive;
    LimelightSubsystem limelightSubsystem;
    double METER_TO_INCH = 39.37008;
    @Override
    public void init() {
        mecanumDrive = new MecanumDrive(this.hardwareMap, new Pose2d(new Vector2d(0, 48), Math.toRadians(270)));
        limelightSubsystem = new LimelightSubsystem(this);
    }
    public void loop() {
        mecanumDrive.updatePoseEstimate();
        double[] limelightPose = limelightSubsystem.getRobotPoseOnField();
        telemetry.addData("RR cood: ", "X: %.3f, Y: %.3f, Heading: %.3f", mecanumDrive.pose.position.x, mecanumDrive.pose.position.y, Math.toDegrees(mecanumDrive.pose.heading.real));
        telemetry.addData("LL cood: ", "X: %.3f, Y: %.3f, Heading: %.3f", limelightPose[0] * METER_TO_INCH, limelightPose[1] * METER_TO_INCH, limelightPose[2]);

        Pose2d LLpose2d = limelightSubsystem.lltoPose2d(limelightSubsystem.getRobotPoseOnField());
        telemetry.addData("Cannot see Apriltag", LLpose2d.equals(limelightSubsystem.ERROR_POSE_2D));
        telemetry.update();
    }
}
