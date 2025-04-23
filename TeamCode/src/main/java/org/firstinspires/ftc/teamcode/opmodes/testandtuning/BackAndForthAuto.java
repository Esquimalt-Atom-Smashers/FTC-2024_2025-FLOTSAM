package org.firstinspires.ftc.teamcode.opmodes.testandtuning;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;

@Autonomous(name = "Test:Back & forth Auto", group = "Push Auto")
public class BackAndForthAuto extends LinearOpMode {
    private MecanumDrive mecanumDrive;
    private LimelightSubsystem limelightSubsystem;
    Pose2d correctPose = new Pose2d(0, 0, 0);
    ElapsedTime LLReactionTimer = new ElapsedTime();
    Boolean firstTimer = true;

    ElapsedTime actionTimer = new ElapsedTime();
    double delays = 0;

    @Override
    public void runOpMode()  {
        Pose2d beginPose = new Pose2d(0,48 , Math.toRadians(0));
        this.mecanumDrive = new MecanumDrive(hardwareMap, beginPose);
        this.limelightSubsystem = new LimelightSubsystem(this);

        waitForStart();
        LLReactionTimer.reset();
        actionTimer.reset();

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                mecanumDrive.actionBuilder(beginPose)
                                        .strafeToConstantHeading(new Vector2d(24, 48))
                                        .strafeToConstantHeading(new Vector2d(0, 48))
                                        .build(),
                                new InstantAction(() -> delays = actionTimer.seconds()),
                                calibrateCoordinate(),
                                new InstantAction(() -> getRobotPose())
                        )
                )
        );
        if (!correctPose.equals(new Pose2d(0, 0, 0))) {
            Actions.runBlocking(
                    new SequentialAction(
                            mecanumDrive.actionBuilder(correctPose)
                                .strafeToLinearHeading(new Vector2d(0, 48), Math.toRadians(0))
                                .build(),
                            new ParallelAction(
                                    new SleepAction(999),
                                    new InstantAction(() -> getRobotPose())
                                    )
                            )
            );
        }
    }


    private void getRobotPose() {
        if (firstTimer) {
            delays = LLReactionTimer.seconds() - delays;
            firstTimer = false;
        }

        mecanumDrive.updatePoseEstimate();
        double[] limelightPose = limelightSubsystem.getRobotPoseOnField();
        telemetry.addData("RR cood: ", "X: %.3f, Y: %.3f, Heading: %.3f", mecanumDrive.pose.position.x, mecanumDrive.pose.position.y, Math.toDegrees(mecanumDrive.pose.heading.real));
        telemetry.addData("LL cood: ", "X: %.3f, Y: %.3f, Heading: %.3f", limelightPose[0] * limelightSubsystem.METER_TO_INCH, limelightPose[1] * limelightSubsystem.METER_TO_INCH, limelightPose[2]);
        telemetry.addData("waited time" , delays);
        telemetry.update();
    }

    public class CalibrateCoordinate implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double[] limelightPose = limelightSubsystem.getRobotPoseOnField();
            double LLX = limelightPose[0] * limelightSubsystem.METER_TO_INCH;
            double LLY = limelightPose[1] * limelightSubsystem.METER_TO_INCH;
            if (LLX != 10000 && LLY != 10000) {
                correctPose = new Pose2d(LLX, LLY, Math.toRadians(limelightPose[2]));
            }
            return correctPose.equals(new Pose2d(0, 0, 0));
        }
    }

    public Action calibrateCoordinate() {
        return new CalibrateCoordinate();
    }
}