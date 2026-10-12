package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;

public class KalmanFilter {

    private double estimateXVariance;

    private double estimateYVariance;

    private Pose estimateState;

    public KalmanFilter(Pose startPose, double initialXVar, double initialYVar) {

        this.estimateState = startPose;

        this.estimateXVariance = initialXVar;

        this.estimateYVariance = initialYVar;

    }


    public void predict(Vector2D velocity, double dt) {

        // Grow uncertainty over time based on velocity

        estimateXVariance += dt * dt * Math.abs(velocity.x());

        estimateYVariance += dt * dt * Math.abs(velocity.y());



        // Advance position prediction

        estimateState = new Pose(

                estimateState.x() + velocity.x() * dt,

                estimateState.y() + velocity.y() * dt,

                estimateState.heading()

        );

    }



    public void update(Pose measurement, double measurementVariance, double heading) {

        // Calculate Kalman Gain for X and Y

        double kalmanGainX = estimateXVariance / (estimateXVariance + measurementVariance);

        double kalmanGainY = estimateYVariance / (estimateYVariance + measurementVariance);



        // Update variances

        estimateXVariance = (1 - kalmanGainX) * estimateXVariance;

        estimateYVariance = (1 - kalmanGainY) * estimateYVariance;



        // Blend prediction with measurement

        double correctedX = estimateState.x() + kalmanGainX * (measurement.x() - estimateState.x());

        double correctedY = estimateState.y() + kalmanGainY * (measurement.y() - estimateState.y());



        estimateState = new Pose(correctedX, correctedY, heading);

    }

    public Pose getState() {
        return estimateState;
    }

}