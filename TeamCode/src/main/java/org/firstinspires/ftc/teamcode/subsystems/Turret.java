
package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Turret {

    private final Servo turretServo;

    // Replace with measured servo calibration.
    private static final double MIN_ANGLE = -Math.PI;
    private static final double MAX_ANGLE = Math.PI;

    // Servo position corresponding to zero turret angle.
    private static final double CENTER_POSITION = 0.5;

    // Calibrate this sign for your physical mounting.
    private static final double DIRECTION = 1.0;

    // Turret pivot offset from robot localization center,
    // in robot-relative inches.
    private static final double PIVOT_X = 0.0;
    private static final double PIVOT_Y = 0.0;

    private double targetAngle = 0;
    private boolean targetReachable = false;

    public Turret(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(Servo.class, "turret");
    }

    public void aimAt(
            double goalX,
            double goalY,
            Pose robotPose) {

        double heading = robotPose.heading();

        // Rotate turret pivot offset into field coordinates.
        double pivotFieldX =
                robotPose.x()
                        + PIVOT_X * Math.cos(heading)
                        - PIVOT_Y * Math.sin(heading);

        double pivotFieldY =
                robotPose.y()
                        + PIVOT_X * Math.sin(heading)
                        + PIVOT_Y * Math.cos(heading);

        double fieldAngle = Math.atan2(
                goalY - pivotFieldY,
                goalX - pivotFieldX
        );

        double relativeAngle = fieldAngle - heading;

        // Normalize to [-PI, PI].
        relativeAngle = Math.atan2(
                Math.sin(relativeAngle),
                Math.cos(relativeAngle)
        );

        targetAngle = relativeAngle;

        targetReachable =
                targetAngle >= MIN_ANGLE &&
                        targetAngle <= MAX_ANGLE;

        if (!targetReachable) {
            return;
        }

        double halfRange = (MAX_ANGLE - MIN_ANGLE) / 2.0;

        double servoPosition =
                CENTER_POSITION
                        + DIRECTION * targetAngle / (2.0 * halfRange);

        servoPosition = Range.clip(
                servoPosition, 0.0, 1.0
        );

        turretServo.setPosition(servoPosition);
    }

    public double getTargetAngleDegrees() {
        return Math.toDegrees(targetAngle);
    }

    public boolean isTargetReachable() {
        return targetReachable;
    }
}
