
package org.firstinspires.ftc.teamcode.teleOp;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.vision.Limelight;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.util.RobotLocalizer;

@TeleOp(name = "Turret Kalman Tracking")
public class turretTeleOp extends OpMode {

    private Drive drive;
    private Limelight limelight;
    private RobotLocalizer localizer;
    private Turret turret;

    // REPLACE with actual field starting pose.
    private static final Pose START_POSE =
            new Pose(0, 0, 0);

    // REPLACE with actual goal coordinates in inches.
    private static final double GOAL_X = 0;
    private static final double GOAL_Y = 0;

    // Enable only after completing calibration.
    private static final boolean ENABLE_TURRET = false;
    private static final boolean ENABLE_VISION = false;

    @Override
    public void init() {

        drive = new Drive(hardwareMap, START_POSE);

        limelight = new Limelight(
                hardwareMap, 0);

        localizer = new RobotLocalizer(
                drive,
                limelight,
                START_POSE
        );

        localizer.setVisionFusionEnabled(ENABLE_VISION);

        turret = new Turret(hardwareMap);

        telemetry.addLine("Turret Tracking Initialized");
        telemetry.addLine("Verify calibration before enabling.");
        telemetry.update();
    }

    @Override
    public void loop() {

        // ---------------- DRIVER CONTROL ----------------

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        // Driver controls drivetrain independently.
        drive.run(forward, strafe, rotate);

        // ---------------- SENSOR FUSION ----------------

        // Predict using odometry movement.
        // Correct using new vision measurements if enabled.
        localizer.update();

        Pose fusedPose = localizer.getPose();
        Pose odomPose = localizer.getOdometryPose();

        // ---------------- TURRET TRACKING ----------------

        if (ENABLE_TURRET) {
            turret.aimAt(
                    GOAL_X,
                    GOAL_Y,
                    fusedPose
            );
        }

        // ---------------- TELEMETRY ----------------

        telemetry.addLine("KALMAN LOCALIZATION");

        telemetry.addData(
                "Odometry X", "%.2f", odomPose.x()
        );

        telemetry.addData(
                "Odometry Y", "%.2f", odomPose.y()
        );

        telemetry.addData(
                "Kalman X", "%.2f", fusedPose.x()
        );

        telemetry.addData(
                "Kalman Y", "%.2f", fusedPose.y()
        );

        telemetry.addData(
                "Heading", "%.2f",
                Math.toDegrees(fusedPose.heading())
        );

        telemetry.addData(
                "Vision Corrected",
                localizer.wasVisionUpdated()
        );

        telemetry.addLine("-------------------------");

        telemetry.addData(
                "Turret Enabled", ENABLE_TURRET
        );

        telemetry.addData(
                "Vision Enabled", ENABLE_VISION
        );

        if (ENABLE_TURRET) {
            telemetry.addData(
                    "Target Angle", "%.2f",
                    turret.getTargetAngleDegrees()
            );

            telemetry.addData(
                    "Target Reachable",
                    turret.isTargetReachable()
            );
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        drive.stop();
        limelight.stop();
    }
}
