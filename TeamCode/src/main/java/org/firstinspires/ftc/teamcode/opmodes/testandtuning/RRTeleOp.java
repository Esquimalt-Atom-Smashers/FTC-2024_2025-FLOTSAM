package org.firstinspires.ftc.teamcode.opmodes.testandtuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

import java.util.ArrayList;
import java.util.List;
@TeleOp(name = "RRTeleOp", group = "zoldprograms")
public class RRTeleOp extends OpMode {
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();

    private DriveSubsystem driveSubsystem;

    @Override
    public void init() {
        this.driveSubsystem = new DriveSubsystem(this);
    }

    @Override
    public void loop() {
        TelemetryPacket packet = new TelemetryPacket();

        // updated based on gamepads
        if (gamepad1.a) {
            runningActions.add(driveSubsystem.toBasket());
        } else {
            driveSubsystem.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        }

        // update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;
        driveSubsystem.getMecanumDrive().updatePoseEstimate();
        dash.sendTelemetryPacket(packet);
    }
}