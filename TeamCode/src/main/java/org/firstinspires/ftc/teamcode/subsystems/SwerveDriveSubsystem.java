package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class SwerveDriveSubsystem extends SubsystemBase {
    private OpMode opMode;

    public SwerveDriveSubsystem(OpMode opMode) {
        this.opMode = opMode;
    }

    public class SwerveDrivePod {
        private HardwareMap hardwareMap;
        private DcMotor driveMotor;
        private Servo turnServo;

        private final double SERVO_MAX_DEGREES = 300.0;
        private final double MIDDLE_POS = 0.5;

        private final double servoOrientation = 0.25 * SERVO_MAX_DEGREES;
        public SwerveDrivePod(HardwareMap hardwareMap) {
            this.hardwareMap = hardwareMap;
            driveMotor = hardwareMap.get(DcMotor.class, "motor");
            turnServo = hardwareMap.get(Servo.class, "servo");

            driveMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public void defaultPos() {
            driveMotor.setPower(0);
            turnServo.setPosition(MIDDLE_POS);
        }

        public void powerToDirection(double angle, double power) {
            double[] swerveVector = swerveVector(angle, power);
            double servoPos = swerveVector[0];
            double motorPower = swerveVector[1];

            turnServo.setPosition(servoPos);
            driveMotor.setPower(power);
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
    }
}

