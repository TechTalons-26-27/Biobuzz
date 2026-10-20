package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;

public class KalmanFilter {

    private double xVariance;

    private double yVariance;

    private Pose filterPose;

    public KalmanFilter(Pose startPose, double xVariance, double yVariance) {
        this.filterPose = startPose;

        this.xVariance = xVariance;

        this.yVariance = yVariance;

    }

    public void predict(Vector2D velocity, double dt) {

        // Grow uncertainty over time based on velocity

        xVariance += dt * dt * Math.abs(velocity.x());

        yVariance += dt * dt * Math.abs(velocity.y());



        // Advance position prediction

        filterPose = new Pose(
            filterPose.x() + velocity.x() * dt,

            filterPose.y() + velocity.y() * dt,

            filterPose.heading()
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