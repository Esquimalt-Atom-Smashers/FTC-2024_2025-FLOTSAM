package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.WristSubsystem;

public class CommandManager {
    ArmSubsystem armSubsystem;
    DriveSubsystem driveSubsystem;
    WristSubsystem wristSubsystem;

    public CommandManager(ArmSubsystem armSubsystem, DriveSubsystem driveSubsystem, WristSubsystem wristSubsystem) {
        this.armSubsystem = armSubsystem;
        this.driveSubsystem = driveSubsystem;
        this.wristSubsystem = wristSubsystem;
    }

    public SequentialCommandGroup getToHighBasketPositionCommand() {
        ArmSubsystem.ArmPosition position = ArmSubsystem.ArmPosition.HIGH_OUTTAKE_POSITION;
        double maxLinearPower = 0.8;
        double maxElbowPowerGoingUp = 0.4;
        double maxElbowPowerGoingDown = 0.25;

        double previousElbowMaxPower = armSubsystem.getMaxElbowPower();
        double previousLinearMaxPower = armSubsystem.getMaxLinearPower();

        if (Math.abs(armSubsystem.getElbowPosition() - position.elbowPos) <= ArmSubsystem.TOLERANCE) {
            return new SequentialCommandGroup(
                    new ArmSubsystem.ArmToPositionCommand(armSubsystem, position, maxLinearPower, previousElbowMaxPower),
                    //new RunCommand(() -> wristSubsystem.setWristPosition(WristSubsystem.WristPosition.OUTTAKE)),
                    new WristSubsystem.MoveWristToPositionCommand(wristSubsystem, WristSubsystem.WristPosition.OUTTAKE),
                    new RunCommand(() -> {
                        armSubsystem.setLinearMaxPower(previousLinearMaxPower);
                        armSubsystem.setElbowMaxPower(previousElbowMaxPower);
                    })
            );
        } else {
            return new SequentialCommandGroup(
                    new ArmSubsystem.SlideToPositionCommand(armSubsystem, ArmSubsystem.SLIDE_MIN_POSITION, maxLinearPower),
                    //new RunCommand(() -> wristSubsystem.setWristPosition(WristSubsystem.WristPosition.COLLAPSED)),
                    new WristSubsystem.MoveWristToPositionCommand(wristSubsystem, WristSubsystem.WristPosition.COLLAPSED),
                    new ArmSubsystem.ElbowToPositionCommand(armSubsystem, position.elbowPos, (position.elbowPos < armSubsystem.getElbowPosition()) ? maxElbowPowerGoingDown : maxElbowPowerGoingUp),
                    new ArmSubsystem.SlideToPositionCommand(armSubsystem, position.slidePos, maxLinearPower),
                    //new RunCommand(() -> wristSubsystem.setWristPosition(WristSubsystem.WristPosition.OUTTAKE)),
                    new WristSubsystem.MoveWristToPositionCommand(wristSubsystem, WristSubsystem.WristPosition.OUTTAKE),
                    new RunCommand(() -> {
                        armSubsystem.setLinearMaxPower(previousLinearMaxPower);
                        armSubsystem.setElbowMaxPower(previousElbowMaxPower);
                    })
            );
        }
    }

    public SequentialCommandGroup getToLowBasketPosition() {
        ArmSubsystem.ArmPosition position = ArmSubsystem.ArmPosition.LOW_OUTTAKE_POSITION;
        double maxLinearPower = 0.8;
        double maxElbowPowerGoingUp = 0.4;
        double maxElbowPowerGoingDown = 0.25;

        double previousElbowMaxPower = armSubsystem.getMaxElbowPower();
        double previousLinearMaxPower = armSubsystem.getMaxLinearPower();

        if (Math.abs(armSubsystem.getElbowPosition() - position.elbowPos) <= ArmSubsystem.TOLERANCE) {
            return new SequentialCommandGroup(
                    //new RunCommand(() -> wristSubsystem.setWristPosition(WristSubsystem.WristPosition.OUTTAKE)),
                    new WristSubsystem.MoveWristToPositionCommand(wristSubsystem, WristSubsystem.WristPosition.OUTTAKE),
                    new ArmSubsystem.ArmToPositionCommand(armSubsystem, position, maxLinearPower, previousElbowMaxPower),
                    new RunCommand(() -> {
                        armSubsystem.setLinearMaxPower(previousLinearMaxPower);
                        armSubsystem.setElbowMaxPower(previousElbowMaxPower);
                    })
            );
        } else {
            return new SequentialCommandGroup(
                    new ArmSubsystem.SlideToPositionCommand(armSubsystem, ArmSubsystem.SLIDE_MIN_POSITION, maxLinearPower),
                    //new RunCommand(() -> wristSubsystem.setWristPosition(WristSubsystem.WristPosition.INTAKE)),
                    new WristSubsystem.MoveWristToPositionCommand(wristSubsystem, WristSubsystem.WristPosition.INTAKE),
                    new ArmSubsystem.ElbowToPositionCommand(armSubsystem, position.elbowPos, (position.elbowPos < armSubsystem.getElbowPosition()) ? maxElbowPowerGoingDown : maxElbowPowerGoingUp),
                    //new RunCommand(() -> wristSubsystem.setWristPosition(WristSubsystem.WristPosition.OUTTAKE)),
                    new WristSubsystem.MoveWristToPositionCommand(wristSubsystem, WristSubsystem.WristPosition.OUTTAKE),
                    new ArmSubsystem.SlideToPositionCommand(armSubsystem, position.slidePos, maxLinearPower),
                    new RunCommand(() -> {
                        armSubsystem.setLinearMaxPower(previousLinearMaxPower);
                        armSubsystem.setElbowMaxPower(previousElbowMaxPower);
                    })
            );
        }
    }

    public SequentialCommandGroup getToHomePosition() {
        ArmSubsystem.ArmPosition position = ArmSubsystem.ArmPosition.INTAKE_POSITION;
        double maxLinearPower = 0.8;
        double maxElbowPowerGoingUp = 0.4;
        double maxElbowPowerGoingDown = 0.25;

        double previousElbowMaxPower = armSubsystem.getMaxElbowPower();
        double previousLinearMaxPower = armSubsystem.getMaxLinearPower();

        if (Math.abs(armSubsystem.getElbowPosition() - position.elbowPos) <= ArmSubsystem.TOLERANCE) {
            return new SequentialCommandGroup(
                    //new RunCommand(() -> wristSubsystem.setWristPosition(WristSubsystem.WristPosition.COLLAPSED)),
                    new WristSubsystem.MoveWristToPositionCommand(wristSubsystem, WristSubsystem.WristPosition.COLLAPSED),
                    new ArmSubsystem.ArmToPositionCommand(armSubsystem, position, maxLinearPower, previousElbowMaxPower),
                    new RunCommand(() -> {
                        armSubsystem.setLinearMaxPower(previousLinearMaxPower);
                        armSubsystem.setElbowMaxPower(previousElbowMaxPower);
                    })
            );
        } else {
            return new SequentialCommandGroup(
                    //new RunCommand(() -> wristSubsystem.setWristPosition(WristSubsystem.WristPosition.READY)),
                    new WristSubsystem.MoveWristToPositionCommand(wristSubsystem, WristSubsystem.WristPosition.READY),
                    new ArmSubsystem.SlideToPositionCommand(armSubsystem, position.slidePos, maxLinearPower),
                    //new RunCommand(() -> wristSubsystem.setWristPosition(WristSubsystem.WristPosition.OUTTAKE)),
                    new WristSubsystem.MoveWristToPositionCommand(wristSubsystem, WristSubsystem.WristPosition.OUTTAKE),
                    new ArmSubsystem.ElbowToPositionCommand(armSubsystem, position.elbowPos, (position.elbowPos < armSubsystem.getElbowPosition()) ? maxElbowPowerGoingDown : maxElbowPowerGoingUp),
                    new RunCommand(() -> {
                        armSubsystem.setLinearMaxPower(previousLinearMaxPower);
                        armSubsystem.setElbowMaxPower(previousElbowMaxPower);
                    })
            );
        }
    }

    public SequentialCommandGroup getToHomePositionHorizontal(){
        ArmSubsystem.ArmPosition position = ArmSubsystem.ArmPosition.INTAKE_POSITION;
        double maxLinearPower = 0.8;
        double maxElbowPowerGoingUp = 0.4;
        double maxElbowPowerGoingDown = 0.25;

        double previousElbowMaxPower = armSubsystem.getMaxElbowPower();
        double previousLinearMaxPower = armSubsystem.getMaxLinearPower();
        return new SequentialCommandGroup(
                //new RunCommand(() -> wristSubsystem.setWristPosition(WristSubsystem.WristPosition.READY)),
                new WristSubsystem.MoveWristToPositionCommand(wristSubsystem, WristSubsystem.WristPosition.READY),
                new ArmSubsystem.ArmToPositionCommand(armSubsystem, position, maxLinearPower, previousElbowMaxPower),
                new RunCommand(() -> {
                    armSubsystem.setLinearMaxPower(previousLinearMaxPower);
                    armSubsystem.setElbowMaxPower(previousElbowMaxPower);
                })
        );
    }
}