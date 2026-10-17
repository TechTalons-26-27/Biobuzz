package org.firstinspires.ftc.teamcode.teleOp;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


/*
Brogan Video Notes
- The further away the robot is from the target position, the faster the turret must move to get to that position.
- The closer the robot is from the target position the slower the turret needs to move
We use PD for turret
- Derivative allows for dampening force when the turret goes faster.
    - Based off of time. So if there is a large gap between two readings, its likely the robot is far away cuz its changing positions really fast
    - apply less dampening force.
    - If less gap, proportional force slows down, dampening goes up
- Proportional allows for proper power to be enforced relative to distances away from the target pos. Farther away = more power, closer = less power

aT = elapsedTime
resetTime

 */
public class turretBasic {

    private DcMotor turret;

    private double kP = 0.0001;
    private double kD = 0.0000;

    private boolean hasLastError = false;

    private double targetX = 0;
    private double targetY = 0;
    private double goalX = 0;
    private double angleTolerance = 0.2;
    private final double MAX_POWER = 0.0;
    private double power = 0;
    private double lastError = 0;
    private final ElapsedTime timer = new ElapsedTime();
    public void init(HardwareMap hwMap) {
        turret = hwMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setkP(double newKP) {
        kP = newKP;
    }

    public void setkD(double newkD) {
        kD = newkD;
    }
    public double getkP() {
        return kP;
    }
    public double getkD() {
        return kD;
    }
    public void resetTimer() {
        timer.reset();
    }

    public void update(Pose robotPose, double currentTurretAngle) {
        double deltaTime = timer.seconds();
        timer.reset();

        //check if robot pose is available
        if (robotPose == null ||
                !Double.isFinite(robotPose.x()) ||
                !Double.isFinite(robotPose.y()) ||
                !Double.isFinite(robotPose.heading()) ||
                !Double.isFinite(currentTurretAngle)) {

            turret.setPower(0);
            lastError = 0;
            return;
        }

        //PD controller
        double targetAngle = Math.atan2(
                targetY - robotPose.y(),
                targetX - robotPose.x()
        );

        double desiredAngle = targetAngle - robotPose.heading();

        double error = AngleUnit.normalizeRadians(
                desiredAngle - currentTurretAngle
        );

        double pTerm = error * kP;

        double dTerm = 0;
        if (deltaTime > 0 && hasLastError) {
            dTerm = (AngleUnit.normalizeRadians(error - lastError)
                    / deltaTime) * kD;
        }

        if (Math.abs(error) < angleTolerance) {
            power = 0;
        } else {
            power = Range.clip(pTerm + dTerm, -MAX_POWER, MAX_POWER);
        }

        // Check turret mechanical limits before applying power
        turret.setPower(power);
        lastError = error;
        hasLastError = true;

    }
}