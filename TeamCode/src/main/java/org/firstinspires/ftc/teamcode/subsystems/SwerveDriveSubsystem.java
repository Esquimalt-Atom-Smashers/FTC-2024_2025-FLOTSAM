package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SwerveDriveSubsystem extends SubsystemBase {
    private OpMode opMode;
    private SwerveDrivePod frontLeft;
    private String FRONT_LEFT_MOTOR_NAME = "frontLeftMotor";
    private String FRONT_LEFT_SERVO_NAME = "frontLeftTurnServo";
    private double FRONT_LEFT_FRONT_POSE = 0.25;

    private SwerveDrivePod rearRight;
    private String REAR_RIGHT_MOTOR_NAME = "rearRightMotor";
    private String REAR_RIGHT_SERVO_NAME = "rearRightTurnServo";
    private double REAR_RIGHT_FRONT_POSE = 0.25;


    public SwerveDriveSubsystem(OpMode opMode) {
        this.opMode = opMode;
        frontLeft = new SwerveDrivePod(opMode.hardwareMap, FRONT_LEFT_MOTOR_NAME, FRONT_LEFT_SERVO_NAME, 300, FRONT_LEFT_FRONT_POSE);
        rearRight = new SwerveDrivePod(opMode.hardwareMap, REAR_RIGHT_MOTOR_NAME, REAR_RIGHT_SERVO_NAME, 300, REAR_RIGHT_FRONT_POSE);
    }

    public void drive(double x, double y, double a) {
        double power = Math.hypot(x, y) + Math.abs(a);
        if (power > 1.0) power = 1.0;

        double angle = Math.toDegrees(Math.atan2(x, y));
        if (power != 0) {
            frontLeft.powerToDirection(angle, power, a);
            rearRight.powerToDirection(angle, power, a);
        } else {
            frontLeft.defaultPos();
            rearRight.defaultPos();
        }
    }

    public class SwerveDrivePod {
        private HardwareMap hardwareMap;
        private DcMotor driveMotor;
        private Servo turnServo;

        private String DRIVE_MOTOR_NAME;
        private String TURN_SERVO_NAME;
        private double SERVO_MAX_DEGREES;
        private double SERVO_FRONT_POSE;
        private final double MIDDLE_POS = 0.5;

        private final double servoOrientation; //== SERVO_FRONT_POSE * SERVO_MAX_DEGREES;

        public SwerveDrivePod(HardwareMap hardwareMap, String driveMotorName, String turnServoName, double servoMaxDegrees, double servoFrontPose) {
            this.hardwareMap = hardwareMap;
            this.DRIVE_MOTOR_NAME = driveMotorName;
            this.TURN_SERVO_NAME = turnServoName;
            this.SERVO_MAX_DEGREES = servoMaxDegrees;
            this.SERVO_FRONT_POSE = servoFrontPose;

            servoOrientation = SERVO_FRONT_POSE * SERVO_MAX_DEGREES;

            driveMotor = hardwareMap.get(DcMotor.class, DRIVE_MOTOR_NAME);
            turnServo = hardwareMap.get(Servo.class, TURN_SERVO_NAME);

            driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public void defaultPos() {
            driveMotor.setPower(0);
            turnServo.setPosition(MIDDLE_POS);
        }

        public void powerToDirection(double angle, double power, double turn) {
            double[] swerveVector = swerveVector(angle + turnAngle(turn) - servoOrientation, power);
            double servoPos = swerveVector[0];
            double motorPower = swerveVector[1];

            turnServo.setPosition(servoPos);
            driveMotor.setPower(motorPower);
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
}

