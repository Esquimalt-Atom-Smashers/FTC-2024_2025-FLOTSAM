package org.firstinspires.ftc.teamcode.opmodes.testandtuning;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenArmSubsystem;

@Autonomous(name = "Test:Back & forth Auto", group = "Push Auto")
public class BackAndForthAuto extends LinearOpMode {
    private MecanumDrive mecanumDrive;
    private LimelightSubsystem limelightSubsystem;

    @Override
    public void runOpMode()  {
        Pose2d beginPose = new Pose2d(48,0 , Math.toRadians(270));
        this.mecanumDrive = new MecanumDrive(hardwareMap, beginPose);
        this.limelightSubsystem = new LimelightSubsystem(this);

        waitForStart();

        Actions.runBlocking(
                mecanumDrive.actionBuilder(beginPose)
                        .strafeToConstantHeading(new Vector2d(48, 24))
                        .strafeToConstantHeading(new Vector2d(48, 0))
                        .build());
    }

    private void getRobotPose() {
        mecanumDrive.updatePoseEstimate();
        double[] limelightPose = limelightSubsystem.getRobotPoseOnField();
        telemetry.addData("RR cood: ", "X: %.3f, Y: %.3f, Heading: %.3f", mecanumDrive.pose.position.x, mecanumDrive.pose.position.y, Math.toDegrees(mecanumDrive.pose.heading.real));
        telemetry.addData("LL cood: ", "X: %.3f, Y: %.3f, Heading: %.3f", limelightPose[0] * limelightSubsystem.METER_TO_INCH, limelightPose[1] * limelightSubsystem.METER_TO_INCH, limelightPose[2]);
    }
}