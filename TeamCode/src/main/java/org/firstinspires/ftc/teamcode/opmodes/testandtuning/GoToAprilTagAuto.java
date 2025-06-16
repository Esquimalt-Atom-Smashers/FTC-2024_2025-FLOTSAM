package org.firstinspires.ftc.teamcode.opmodes.testandtuning;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenArmSubsystem;
@Disabled
@Autonomous(name = "Test:Go to apriltag Auto", group = "Push Auto")
public class GoToAprilTagAuto extends LinearOpMode {
    private MecanumDrive mecanumDrive;
    private LimelightSubsystem limelightSubsystem;

    @Override
    public void runOpMode()  {
        Pose2d beginPose = new Pose2d(0,48 , Math.toRadians(0));
        this.limelightSubsystem = new LimelightSubsystem(this);

        waitForStart();

        Actions.runBlocking(
                new ParallelAction(
                        calibrateCoordinate(new Vector2d(0, 48)),
                        new InstantAction(this::getRobotPose)
                )
        );

    }

    private void getRobotPose() {
        mecanumDrive.updatePoseEstimate();
        double[] limelightPose = limelightSubsystem.getRobotPoseOnField();
        telemetry.addData("RR cood: ", "X: %.3f, Y: %.3f, Heading: %.3f", mecanumDrive.pose.position.x, mecanumDrive.pose.position.y, Math.toDegrees(mecanumDrive.pose.heading.real));
        telemetry.addData("LL cood: ", "X: %.3f, Y: %.3f, Heading: %.3f", limelightPose[0] * limelightSubsystem.METER_TO_INCH, limelightPose[1] * limelightSubsystem.METER_TO_INCH, limelightPose[2]);
        telemetry.update();
    }

    public class CalibrateCoordinate implements Action {
        Vector2d target;
        public CalibrateCoordinate(Vector2d target) {
            this.target = target;
        }
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            mecanumDrive = new MecanumDrive(hardwareMap, new Pose2d(0, 24, Math.toRadians(270)));
            mecanumDrive.actionBuilder( new Pose2d(0, 24, Math.toRadians(270)))
                        .strafeToConstantHeading(target)
                        .build();
            return false;
        }
    }

    public Action calibrateCoordinate(Vector2d target) {
        return new CalibrateCoordinate(target);
    }
}