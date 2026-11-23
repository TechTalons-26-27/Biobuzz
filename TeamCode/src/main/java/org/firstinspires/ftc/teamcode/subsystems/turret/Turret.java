package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Heading;
import org.firstinspires.ftc.teamcode.util.ServoMath;

public class Turret {
    // ---------------- Placeholders: tune these ----------------
    private static final double TURRET_TEETH = 100;
    private static final double DRIVER_TEETH = 20;
    // Turret-angle offset (rad) added to the computed angle to correct mounting/zero error.
    private static final double TURRET_ZERO_OFFSET = 0;
    // -----------------------------------------------------------

    private static final double GEAR_RATIO = TURRET_TEETH / DRIVER_TEETH;
    private static final double SERVO_RANGE = Math.toRadians(1800);
    private static final double TURRET_LIMIT = (SERVO_RANGE / 2) / GEAR_RATIO;

    private final Servo servo;
    private double turretAngle = 0; // last commanded turret angle relative to robot (rad)
    private double servoPosition = 0.5;

    public Turret(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "turret");
        servo.setPosition(servoPosition);
    }


    public void run(Pose currentPose, Pose targetPose) {
        // Field-frame bearing to target, then make it relative to the robot's heading.
        double fieldAngle = Heading.calculateTh(currentPose, targetPose);
        double relative = wrap(fieldAngle - currentPose.heading() + TURRET_ZERO_OFFSET);

        turretAngle = pickReachable(relative);

        // Turret angle -> servo shaft angle; servo center (0.5) = turret angle 0.
        double servoAngle = turretAngle * GEAR_RATIO;
        servoPosition = ServoMath.toPosition(servoAngle, SERVO_RANGE, -SERVO_RANGE / 2);
        servo.setPosition(servoPosition);
    }

    private static double wrap(double rad) {
        while (rad > Math.PI) rad -= 2 * Math.PI;
        while (rad <= -Math.PI) rad += 2 * Math.PI;
        return rad;
    }

    /*
     * If the turret can travel more than a full rotation, several angles point the same way;
     * pick the reachable one closest to the current angle to avoid unnecessary unwinding.
     */
    private double pickReachable(double rel) {
        double best = rel;
        double bestDist = Double.MAX_VALUE;
        boolean found = false;
        for (int k = -2; k <= 2; k++) {
            double cand = rel + k * 2 * Math.PI;
            if (Math.abs(cand) > TURRET_LIMIT) continue;
            double dist = Math.abs(cand - turretAngle);
            if (dist < bestDist) {
                bestDist = dist;
                best = cand;
                found = true;
            }
        }
        // Target is in the dead zone: clamp to the nearest limit.
        return found ? best : Math.max(-TURRET_LIMIT, Math.min(TURRET_LIMIT, rel));
    }

    public double getTurretAngle() {
        return turretAngle;
    }

    public double getServoPosition() {
        return servoPosition;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Turret angle (rad)", turretAngle);
        telemetry.addData("Turret servo pos", servoPosition);
    }
}
