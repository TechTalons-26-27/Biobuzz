package org.firstinspires.ftc.teamcode.teleOp;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.drive.BaseDrive;
import org.firstinspires.ftc.teamcode.subsystems.vision.Limelight;
import org.firstinspires.ftc.teamcode.util.KalmanFilter; // Adjust if named KalmanFilter
import org.firstinspires.ftc.teamcode.util.PoseStorage;

@TeleOp(name="Auto Align Test with Kalman")
public class turretTeleOp extends OpMode {

    private Limelight limelight;
    private BaseDrive drive;
    private KalmanFilter kalmanFilter;

    // --------------------------- PD Controller ---------------------------
    double kP = -0.0200;
    double error = 0;
    double lastError = 0;
    double angleTolerance = 0.2;
    double kD = -0.0005;
    double curTime = 0;
    double lastTime = 0;

    // --------------------------- Drive ---------------------------
    double forward, strafe, rotate;

    // --------------------------- controller based PD tuning ---------------------------
    double[] stepSizes = {0.1, 0.01, 0.001, 0.0001};
    int stepIndex = 2;

    @Override
    public void init() {
        limelight = new Limelight(hardwareMap, telemetry, 0);
        drive = new BaseDrive(hardwareMap);

        // Initialize Kalman Filter with a starting Pose (e.g., 0,0,0) and variances
        kalmanFilter = new KalmanFilter(new Pose(0, 0, 0), 1.0, 1.0);

        telemetry.addLine("Initialized with Kalman Filter");
        telemetry.update();
    }

    @Override
    public void start() {
        resetRuntime();
        curTime = getRuntime();
        lastTime = curTime;
    }

    @Override
    public void loop() {
        // Calculate delta time (dt) for prediction updates
        curTime = getRuntime();
        double dT = curTime - lastTime;
        lastTime = curTime;

        // --------------------------- get mecanum drive inputs ---------------------------
        forward = gamepad1.left_stick_y;
        strafe = -gamepad1.left_stick_x;
        rotate = -gamepad1.right_stick_x;

        // 1. PREDICT: Update Kalman filter model using robot velocity (estimated from sticks or odometry)
        Vector2D estimatedVelocity = Vector2D.cartesian(forward * 10.0, strafe * 10.0); //Scale factor based on robot speed
        kalmanFilter.predict(estimatedVelocity, dT);

        // --------------------------- get limelight input ---------------------------
        double Tx = limelight.getTx();
        double Ty = limelight.getTy();

        // 2. UPDATE: If Limelight sees a target, feed it into the Kalman filter as a measurement
        if (Tx != -999 && Ty != -999) {
            // Construct a measurement pose using Limelight data (mapping Tx/Ty to field coordinates or offsets)
            Pose measurementPose = new Pose(Tx, Ty, 0);
            double measurementVariance = 0.5; // Trust level of the vision measurement

            kalmanFilter.update(measurementPose, measurementVariance, 0);
        }

        // Fetch filtered pose state from the Kalman Filter
        Pose filteredPose = kalmanFilter.getPose();
        PoseStorage.currentPose = filteredPose; // Keep global PoseStorage in sync

        // --------------------------- auto align rotation logic ---------------------------
        if (gamepad1.left_trigger > 0.3) {
            // Use filtered X position or Tx combined with filtered state for aiming error
            error = Tx; // Alternatively: Tx + filteredPose.x() depending on coordinate space setup

            if (Math.abs(error) < angleTolerance) {
                rotate = 0;
            } else {
                double pTerm = error * kP;
                double dTerm = 0;

                if (dT != 0) {
                    dTerm = ((error - lastError) / dT) * kD;
                }

                rotate = Range.clip(pTerm + dTerm, -0.4, 0.4);
                lastError = error;
            }
        } else {
            lastError = 0;
        }

        // drive motors
        drive.run(forward, strafe, rotate);

        //update P and D on the fly
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            kP -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            kP += stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            kD += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            kD -= stepSizes[stepIndex];
        }

        // --------------------------- telemetry ---------------------------
        if (Tx != -999) {
            if (gamepad1.left_trigger > 0.3) {
                telemetry.addLine("AUTO ALIGN (Kalman Fusion Active)");
            }
            telemetry.addData("Error", error);
        } else {
            telemetry.addLine("MANUAL Rotate Mode");
        }
        telemetry.addData("Tx", Tx);
        telemetry.addData("Filtered X", "%.2f", filteredPose.x());
        telemetry.addData("Filtered Y", "%.2f", filteredPose.y());
        telemetry.addLine("-----------------------------");
        telemetry.addData("Tuning P", "%.4f (D-Pad L/R)", kP);
        telemetry.addData("Tuning D", "%.4f (D-Pad U/D)", kD);
        telemetry.addData("Step Sizes", "%.4f (B Button)", stepSizes[stepIndex]);
        telemetry.update();
    }
}