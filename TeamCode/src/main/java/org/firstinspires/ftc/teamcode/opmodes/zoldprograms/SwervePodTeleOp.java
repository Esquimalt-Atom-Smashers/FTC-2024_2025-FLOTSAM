package org.firstinspires.ftc.teamcode.opmodes.zoldprograms;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Single Swerve Pod TeleOp", group = "Test")
public class SwervePodTeleOp extends LinearOpMode {

    // Hardware
    private DcMotor driveMotor;
    private Servo turnServo;

    private final double SERVO_MAX_DEGREES = 300.0;
    private final double MIDDLE_POS = 0.5;
    private final double FORWARD_POS = 0.29;

    private final double servoOrientation = FORWARD_POS * SERVO_MAX_DEGREES;
    @Override
    public void runOpMode() {
        // Initialize hardware
        driveMotor = hardwareMap.get(DcMotor.class, "motor");
        turnServo = hardwareMap.get(Servo.class, "servo");

        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turnServo.setPosition(0.5);


        waitForStart();

        while (opModeIsActive()) {
            // Get joystick input
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y; // Invert Y axis for forward
            double a = gamepad1.right_stick_x;
            double power = Math.hypot(x, y) + Math.abs(a);
            if (power > 1.0) power = 1.0;

            // Calculate target angle (0 to 360)
            double angle = Math.toDegrees(Math.atan2(x, y));
            telemetry.addData("Target Angle", "%.2f°", angle);
            angle = angle - servoOrientation;

            double servoPos;
            double motorPower;

            if (power != 0) {
                double[] swerveVector = swerveVector(angle + turnAngle(a), power);
                servoPos = swerveVector[0] / SERVO_MAX_DEGREES;
                motorPower = swerveVector[1];
//                servoPos = angle/SERVO_MAX_DEGREES + 0.5;
//                motorPower = power;
            } else {
                servoPos = 0.5;
                motorPower = 0;
            }
            // Apply outputs
            turnServo.setPosition(servoPos);
//            driveMotor.setPower(power);

            // Telemetry
            telemetry.addData("Joystick X/Y", "%.2f / %.2f", x, y);
            telemetry.addData("Servo Position", "%.2f", servoPos);
            telemetry.addData("Motor Power", "%.2f", motorPower);
            telemetry.update();
        }
    }

    private double[] swerveVector(double angleDegrees, double power) {
        double servoPos = angleDegrees / SERVO_MAX_DEGREES + MIDDLE_POS;
        if (servoPos < MIDDLE_POS + (90 / SERVO_MAX_DEGREES) && servoPos > MIDDLE_POS - (90 / SERVO_MAX_DEGREES)) {
            return new double[] {servoPos, power};
        } else {
            servoPos = angleDegrees > 0 ? ((angleDegrees - 180) / SERVO_MAX_DEGREES) + MIDDLE_POS : ((angleDegrees + 180) / SERVO_MAX_DEGREES) + MIDDLE_POS;
            return new double[] {servoPos, -power};
        }
    }

    private double turnAngle(double a) {
        if (a == 0) return 0;
        if (a > 0) return 90;
        else return -90;
    }
}
