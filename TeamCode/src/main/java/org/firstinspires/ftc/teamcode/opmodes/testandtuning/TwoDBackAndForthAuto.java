package org.firstinspires.ftc.teamcode.opmodes.testandtuning;

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

@Autonomous(name = "Test:2d Back & forth Auto", group = "Push Auto")
public class TwoDBackAndForthAuto extends LinearOpMode {
    private MecanumDrive mecanumDrive;
    private LimelightSubsystem limelightSubsystem;
    Pose2d correctPose;
    ElapsedTime LLReactionTimer = new ElapsedTime();
    Boolean firstTimer = true;

    ElapsedTime actionTimer = new ElapsedTime();
    double delays = 0;

    @Override
    public void runOpMode()  {
        Pose2d beginPose = new Pose2d(0,48 , Math.toRadians(0));
        this.mecanumDrive = new MecanumDrive(hardwareMap, beginPose);
        this.limelightSubsystem = new LimelightSubsystem(this);
        correctPose = limelightSubsystem.errorPose2d;

        waitForStart();
        LLReactionTimer.reset();
        actionTimer.reset();

        Actions.runBlocking(
                new ParallelAction(
                        new SequentialAction(
                                mecanumDrive.actionBuilder(beginPose)
                                        .splineToLinearHeading(new Pose2d(15, 15, Math.toRadians(180)),  Math.toRadians(180))
                                        .strafeToLinearHeading(new Vector2d(0, 48), Math.toRadians(0))
                                        .build()
                        )
                )
        );
        delays = actionTimer.seconds();
        correctPose = limelightSubsystem.getLLCoorInAutoBlocking();
        getRobotPose();
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
}
