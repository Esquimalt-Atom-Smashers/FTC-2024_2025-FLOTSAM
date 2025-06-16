package org.firstinspires.ftc.teamcode.opmodes.testandtuning;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.Arrays;
import java.util.List;
@Disabled
@TeleOp(name="ManualTurning", group="Robot")
public class ManualTurning extends LinearOpMode {
    DriveSubsystem driveSubsystem;
    Limelight3A limelight;
    @Override
    public void runOpMode() throws InterruptedException {
        driveSubsystem = new DriveSubsystem(this);

        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.dpad_up) {driveSubsystem.turnTo(0);
            } else if (gamepad1.dpad_right) {driveSubsystem.turnTo(-90);
            } else if (gamepad1.dpad_down) {driveSubsystem.turnTo(180);
            } else if (gamepad1.dpad_left) {driveSubsystem.turnTo(90);}
            else driveSubsystem.stopAll();

            telemetry.addData("Heading", driveSubsystem.getHeading());
            telemetry.update();
        }
    }
}
