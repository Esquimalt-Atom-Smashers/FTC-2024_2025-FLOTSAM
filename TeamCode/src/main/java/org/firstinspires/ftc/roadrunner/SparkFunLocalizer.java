package org.firstinspires.ftc.roadrunner;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.roadrunner.messages.ThreeDeadWheelInputsMessage;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public final class SparkFunLocalizer implements Localizer{
    SparkFunOTOS opticalOdometry;

    private double lastX, lastY, lastTheta;
    private boolean initialized;

    public SparkFunLocalizer(HardwareMap hardwareMap) {
        opticalOdometry = hardwareMap.get(SparkFunOTOS.class, "otos");

        opticalOdometry.setLinearUnit(DistanceUnit.INCH);
        opticalOdometry.setAngularUnit(AngleUnit.RADIANS);
        SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, 0);
        opticalOdometry.setOffset(offset);

        opticalOdometry.setLinearScalar(1.0);
        opticalOdometry.setAngularScalar(1.0);

        opticalOdometry.calibrateImu();
        opticalOdometry.resetTracking();

        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        opticalOdometry.setPosition(currentPosition);
    }

    public Twist2dDual<Time> update() {
        SparkFunOTOS.Pose2D velocity2d = opticalOdometry.getVelocity();

        if (!initialized) {
            initialized = true;
            lastX = velocity2d.x;
            lastY = velocity2d.y;
            lastTheta = velocity2d.h;

            return new Twist2dDual<>(
                    Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                    DualNum.constant(0.0, 2)
            );
        }

        SparkFunOTOS.Pose2D pose2D = opticalOdometry.getPosition();
        double xDelta = pose2D.x - lastX;
        double yDelta = pose2D.y - lastY;
        double aDelta = pose2D.h - lastTheta;

        double xV = velocity2d.x;
        double yV = velocity2d.y;
        double aV = velocity2d.h;

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {xDelta, xV}),
                        new DualNum<Time>(new double[] {yDelta, yV})
                ),
                new DualNum<>(new double[] {aDelta, aV})
        );

        lastX = pose2D.x;
        lastY = pose2D.y;
        lastTheta = pose2D.h;

        return twist;
    }
}
