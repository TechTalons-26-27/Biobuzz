
package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.vision.Limelight;
import com.pedropathing.math.Vector2D;
import com.qualcomm.robotcore.util.ElapsedTime;

public class RobotLocalizer {

    private final ElapsedTime timer = new ElapsedTime();
    private final Drive drive;
    private final Limelight limelight;
    private final KalmanFilter filter;

    private Pose lastOdometryPose;

    // Keep false until Limelight coordinates are verified.
    private boolean visionFusionEnabled = false;

    // Example vision variance in square inches.
    private double visionVariance = 4.0;

    private boolean visionUpdated = false;

    public RobotLocalizer(
            Drive drive,
            Limelight limelight,
            Pose startPose) {

        this.drive = drive;
        this.limelight = limelight;

        filter = new KalmanFilter(startPose, 1.0, 1.0);
        lastOdometryPose = drive.getOdometryPose();
        timer.reset();
    }

    public void update() {

        Pose odom = drive.getOdometryPose();

        double dx = odom.x() - lastOdometryPose.x();
        double dy = odom.y() - lastOdometryPose.y();

        double dt = timer.seconds();
        timer.reset();

        if (dt > 0) {
            Vector2D velocity = Vector2D.cartesian(
                    dx / dt,
                    dy / dt
            );

            filter.predict(velocity, dt, odom.heading());
        }

        lastOdometryPose = odom;
        visionUpdated = false;

        if (visionFusionEnabled) {

            Pose visionPose = limelight.getNewFieldPose();

            if (visionPose != null) {

                Pose predicted = filter.getPose();

                double residual = Math.hypot(
                        visionPose.x() - predicted.x(),
                        visionPose.y() - predicted.y()
                );

                // Preliminary outlier rejection.
                if (residual < 12.0) {
                    filter.update(
                            visionPose,
                            visionVariance,
                            odom.heading()
                    );
                    visionUpdated = true;
                }
            }
        }
    }

    public Pose getPose() {
        return filter.getPose();
    }

    public Pose getOdometryPose() {
        return drive.getOdometryPose();
    }

    public boolean wasVisionUpdated() {
        return visionUpdated;
    }

    public void setVisionFusionEnabled(boolean enabled) {
        visionFusionEnabled = enabled;
    }
}
