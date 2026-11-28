package org.firstinspires.ftc.teamcode.subsystems.outtake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.HardwareNames;

/**
 * Flywheel only. Uses the hub's built-in velocity PIDF (RUN_USING_ENCODER + setVelocity),
 * so the speed holds as the battery drains. Feeding is handled by Shooter.
 */
public class Outtake {

    // Encoder ticks per motor revolution (28 = bare motor shaft encoder). RPM below is motor RPM.
    private static final double TICKS_PER_REV = 28.0;

    // TODO: TUNE with the outtake tuner opmode. Start F at 32767 / (max ticks per second at full power).
    private static final double P = 10;
    private static final double F = 11.7;

    // RPM the flywheel holds while not shooting, so spin-up is short. TODO: TUNE
    public static final double IDLE_RPM = 1000;

    // Feeding is allowed only while |current - target| is within this. TODO: TUNE
    private static final double RPM_TOLERANCE = 50;

    private final DcMotorEx motor;
    private double targetRPM = 0;

    public Outtake(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, HardwareNames.OUTTAKE);
        motor.setDirection(DcMotor.Direction.REVERSE);
        // FLOAT so the flywheel coasts down instead of braking hard.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(P, 0, 0, F));
        setTargetRPM(0); // never move during init
    }

    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
        motor.setVelocity(rpm * TICKS_PER_REV / 60.0);
    }

    /** Sets the target RPM from the shot table for this distance to the goal (inches). */
    public void aimAt(double distance) {
        setTargetRPM(ShotTable.rpmFor(distance));
    }

    public void idle() {
        setTargetRPM(IDLE_RPM);
    }

    public void stop() {
        setTargetRPM(0);
    }

    public double getRPM() {
        return motor.getVelocity() * 60.0 / TICKS_PER_REV;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public boolean isAtSpeed() {
        return targetRPM > 0 && Math.abs(getRPM() - targetRPM) < RPM_TOLERANCE;
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Flywheel target RPM", "%.0f", targetRPM);
        telemetry.addData("Flywheel RPM", "%.0f", getRPM());
        telemetry.addData("Flywheel at speed", isAtSpeed());
    }
}