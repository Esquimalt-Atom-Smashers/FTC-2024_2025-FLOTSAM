package org.firstinspires.ftc.teamcode.opmodes.zoldprograms;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class SwervePodTeleOp extends OpMode {
    DcMotorEx motor;
    Servo servo;

    double SERVO_RANGE = 330;
    double SEREWE_ORIENTATION = -45;
    @Override
    public void init() {
        motor = hardwareMap.get(DcMotorEx.class, "motor");
        servo = hardwareMap.get(Servo.class, "servo");
    }

    public void loop() {
        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;
        double power = Range.clip(Math.sqrt(Math.pow(-gamepad1.left_stick_y, 2) + Math.pow(gamepad1.left_stick_x, 2)), -1, 1);
        directionControl(drive, strafe, turn);
        motor.setPower(power);
    }

    public void directionControl(double drive, double strafe, double turn) {
        double forwardDegree = SERVO_RANGE - SEREWE_ORIENTATION;
        double direction = Math.atan(strafe / drive);
        servo.setPosition(forwardDegree + direction);
    }
}
