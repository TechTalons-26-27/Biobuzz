package org.firstinspires.ftc.teamcode.config.subsystems.outtake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class outtakeLogic {

    // ---------------- Constants ----------------
    private static final double TICKS_PER_REV = 28.0;
    public OuttakeState outtakeState = OuttakeState.IDLE;
    // ---------------- Hardware ----------------
    private DcMotorEx outtakeMotor;
    private DcMotorEx stageMotor;
    private DcMotorEx intakeMotor;
    private ElapsedTime outtakeTimer = new ElapsedTime();
    private ElapsedTime manualTimer = new ElapsedTime();
    private int shotsRemaining = 0;
    private double idleRPM = 500;     // flywheel RPM when idle
    private double targetRPM = 3000;  // flywheel target RPM
    private double minRPM = 2700;      // minimum RPM to start firing
    private double maxSpinupTime = 5.0;    // max wait for flywheel spinup
    private double stageShootTime = 1;  // stage forward time

    // ---------------- PIDF ----------------
    private double P = 0;
    private double F = 24;

    public void init(HardwareMap hardwareMap) {
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtake");
        stageMotor = hardwareMap.get(DcMotorEx.class, "stage");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");

        outtakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        outtakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        stageMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        //outtakeMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
        // new PIDFCoefficients(P, 0, 0, F));

        /// outtakeState = OuttakeState.IDLE;
    }

    public void start() {
        // Start idle behavior here, legal in competition
        stageMotor.setPower(0);  // idle reverse
        setVelocityRPM(idleRPM);
        outtakeState = OuttakeState.IDLE;
    }

    // ---------------- Main update loop ----------------
    public void update() {
        double currentRPM = (outtakeMotor.getVelocity() * 60.0) / TICKS_PER_REV;

        switch (outtakeState) {

            case IDLE:
                setVelocityRPM(idleRPM);

                // Immediately transition if shots are requested
                if (shotsRemaining > 0) {
                    intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
                    stageMotor.setDirection(DcMotorEx.Direction.REVERSE);
                    intakeMotor.setPower(1);
                    stageMotor.setPower(1);
                    setVelocityRPM(targetRPM);
                    outtakeTimer.reset();
                    outtakeState = OuttakeState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                // Wait for flywheel to reach minRPM or max spinup time
                if (/*currentRPM >= minRPM ||*/ outtakeTimer.seconds() >= maxSpinupTime) {
                    intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
                    stageMotor.setDirection(DcMotorEx.Direction.FORWARD);
                    intakeMotor.setPower(1);
                    stageMotor.setPower(1);
                    outtakeTimer.reset();
                    outtakeState = OuttakeState.LAUNCH;
                }
                break;

            case LAUNCH:
                // Flywheel keeps spinning
                setVelocityRPM(targetRPM);

                if (outtakeTimer.seconds() >= stageShootTime) {
                    shotsRemaining--;
                    outtakeTimer.reset();
                    outtakeState = OuttakeState.RESET_STAGE;
                }
                break;

            case RESET_STAGE:
                if (shotsRemaining > 0) {
                    outtakeTimer.reset();
                    outtakeState = OuttakeState.SPIN_UP;
                } else {
                    intakeMotor.setPower(0);
                    stageMotor.setPower(0);
                    setVelocityRPM(idleRPM);
                    outtakeState = OuttakeState.IDLE;
                }
                break;
        }
    }

    // ---------------- Public API ----------------
    public void fireShots(int numShots) {
        if (numShots <= 0) return;

        shotsRemaining = numShots;

        // Immediately start SPIN_UP if currently idle
//        if (outtakeState == OuttakeState.IDLE) {
//            setVelocityRPM(targetRPM);
//            outtakeTimer.reset();
//            outtakeState = OuttakeState.SPIN_UP;
//        }
    }

    public boolean isBusy() {
        return outtakeState != OuttakeState.IDLE;
    }

    // ---------------- Helpers ----------------
    private void setVelocityRPM(double rpm) {
        double ticksPerSecond = (rpm * TICKS_PER_REV) / 60.0;
        outtakeMotor.setVelocity(ticksPerSecond);
    }

    public double getCurrentRPM() {
        return (outtakeMotor.getVelocity() * 60.0) / TICKS_PER_REV;
    }

    public OuttakeState getState() {
        return outtakeState;
    }

    public int getShotsRemaining() {
        return shotsRemaining;
    }

    public double getTargetVelocity() {
        return (targetRPM * TICKS_PER_REV) / 60.0;
    }

    public enum OuttakeState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        RESET_STAGE
    }
}