package org.firstinspires.ftc.teamcode.opmodes.zoldprograms;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Single Swerve Pod TeleOp", group = "Test")
public class SwervePodTeleOp extends LinearOpMode {

    // Hardware
    private DcMotor driveMotor;
    private Servo turnServo;

    // Servo parameters (for ~330° servo)
    private final double SERVO_MIN_DEGREES = 0.0;
    private final double SERVO_MAX_DEGREES = 330.0;

    @Override
    public void runOpMode() {
        // Initialize hardware
        driveMotor = hardwareMap.get(DcMotor.class, "motor");
        turnServo = hardwareMap.get(Servo.class, "servo");

        driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        while (opModeIsActive()) {
            // Get joystick input
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y; // Invert Y axis for forward
            double power = Math.hypot(x, y);
            if (power > 1.0) power = 1.0;

            // Calculate target angle (0 to 360)
            double angle = Math.toDegrees(Math.atan2(y, x));
            angle = normalizeAngle(angle);

            // Convert to servo position (0.0 to 1.0)
            double servoPos = angleToServoPosition(angle);

            // Apply outputs
            turnServo.setPosition(servoPos);
            driveMotor.setPower(power);

            // Telemetry
            telemetry.addData("Joystick X/Y", "%.2f / %.2f", x, y);
            telemetry.addData("Target Angle", "%.2f°", angle);
            telemetry.addData("Servo Position", "%.2f", servoPos);
            telemetry.addData("Motor Power", "%.2f", power);
            telemetry.update();
        }
    }

    private double normalizeAngle(double angle) {
        angle %= 360;
        if (angle < 0) angle += 360;
        return angle;
    }

    private double angleToServoPosition(double angleDegrees) {
        // Clamp to max physical range
        angleDegrees = Math.max(SERVO_MIN_DEGREES, Math.min(angleDegrees, SERVO_MAX_DEGREES));
        return angleDegrees / SERVO_MAX_DEGREES;
    }
}
