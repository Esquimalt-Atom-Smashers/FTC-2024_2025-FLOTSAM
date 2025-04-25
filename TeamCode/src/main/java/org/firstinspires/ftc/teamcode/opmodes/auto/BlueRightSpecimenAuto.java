package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.SpecimenArmSubsystem;

@Autonomous(name = "Test:BlueRightSpecimenAuto", group = "Scoring Auto")
public class BlueRightSpecimenAuto extends LinearOpMode {
    private SpecimenArmSubsystem specimenArmSubsystem;

    @Override
    public void runOpMode()  {
        Pose2d beginPose = new Pose2d(-7.125,63.25 , Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);
        this.specimenArmSubsystem = new SpecimenArmSubsystem(this);

        waitForStart();

        Actions.runBlocking (
                drive.actionBuilder(beginPose)
                        .stopAndAdd(specimenArmSubsystem.CloseClaw())
                        .stopAndAdd(specimenArmSubsystem.ScoreSpecimen())
                        .strafeToLinearHeading(new Vector2d(5,28), Math.toRadians(180))
                        .stopAndAdd(specimenArmSubsystem.OpenClaw())
//get sample
                        .strafeToConstantHeading( new Vector2d(-35.75, 53) )
                        .strafeToConstantHeading(new Vector2d(-35.75,30))

                        .stopAndAdd(new SleepAction(0.2))
                        .stopAndAdd(specimenArmSubsystem.PutDown())
                        .stopAndAdd(specimenArmSubsystem.OpenClaw())

                        .splineToConstantHeading(new Vector2d(-47.6,17), Math.toRadians(180))
                        .strafeToConstantHeading(new Vector2d( -47.6, 61))
                        .strafeToLinearHeading(new Vector2d(-47.6, 48), Math.toRadians(180))

                        .stopAndAdd(specimenArmSubsystem.WallPos())
                        .stopAndAdd(specimenArmSubsystem.OpenClaw())
//get sec spec
                        .setTangent(Math.toRadians(270))
                        .splineToConstantHeading(new Vector2d(-55, 71), Math.toRadians(90))
                        .strafeToLinearHeading(new Vector2d(-40, 71), Math.toRadians(184))

                        .stopAndAdd(specimenArmSubsystem.CloseClaw())
                        .stopAndAdd(new SleepAction(0.3))
                        .stopAndAdd(specimenArmSubsystem.CloseClaw())
                        .stopAndAdd(specimenArmSubsystem.LiftPos())
                        .strafeToLinearHeading(new Vector2d(-40, 60), Math.toRadians(180))

                        .stopAndAdd(specimenArmSubsystem.ScoreSpecimen())
                        .setTangent(0)
                        .splineToLinearHeading(new Pose2d(new Vector2d(-5,30), Math.toRadians(180)), Math.toRadians(270))
//get third spec
                        .setTangent(Math.toRadians(90))
                        .splineToLinearHeading(new Pose2d(new Vector2d(-8,50),Math.toRadians(180)),Math.toRadians(90))

                        .stopAndAdd(specimenArmSubsystem.OpenClaw())
                        .stopAndAdd(specimenArmSubsystem.WallPos())
                        .splineToConstantHeading(new Vector2d(-55, 71), Math.toRadians(90))
                        .strafeToLinearHeading(new Vector2d(-40, 71), Math.toRadians(184))

                        .stopAndAdd(specimenArmSubsystem.CloseClaw())
                        .stopAndAdd(new SleepAction(0.3))
                        .stopAndAdd(specimenArmSubsystem.LiftPos())


                        .strafeToLinearHeading(new Vector2d(-40, 60), Math.toRadians(180))
                        .stopAndAdd(specimenArmSubsystem.ScoreSpecimen())

                        .setTangent(0)
                        .splineToLinearHeading(new Pose2d(new Vector2d(-8,30), Math.toRadians(180)), Math.toRadians(270))

                        .setTangent(Math.toRadians(90))
                        .splineToConstantHeading(new Vector2d(-38, 70), Math.toRadians(180))
                        .build());


    }
}