package org.firstinspires.ftc.teamcode.localization;

import com.pedropathing.math.Pose;

public class KalmanFilter {

    // TODO: TUNE Process noise added to the variance (in^2).
    private static final double DRIFT_NOISE = 0.05;

    private double xVariance;
    private double yVariance;

    private Pose filterPose;

    public KalmanFilter(Pose startPose, double xVariance, double yVariance) {
        this.filterPose = startPose;

        this.xVariance = xVariance;
        this.yVariance = yVariance;

    }


    public void predict(double dx, double dy, double dt, double heading) {

        // Uncertainty grows with distance travelled (odometry drift). Independent of loop rate.
        xVariance += DRIFT_NOISE * Math.abs(dx);
        yVariance += DRIFT_NOISE * Math.abs(dy);

        filterPose = new Pose(
                filterPose.x() + dx,
                filterPose.y() + dy,
                heading
        );
    }

    public void update(Pose measurement, double measurementVariance, double heading) {

        // Calculate Kalman Gain for X and Y
        double kalmanGainX = xVariance / (xVariance + measurementVariance);
        double kalmanGainY = yVariance / (yVariance + measurementVariance);

        // Update variances
        xVariance = (1 - kalmanGainX) * xVariance;
        yVariance = (1 - kalmanGainY) * yVariance;



        // Blend prediction with measurement
        double correctedX = filterPose.x() + kalmanGainX * (measurement.x() - filterPose.x());
        double correctedY = filterPose.y() + kalmanGainY * (measurement.y() - filterPose.y());

        filterPose = new Pose(correctedX, correctedY, heading);
    }

    public Pose getPose() {
        return filterPose;
    }
}