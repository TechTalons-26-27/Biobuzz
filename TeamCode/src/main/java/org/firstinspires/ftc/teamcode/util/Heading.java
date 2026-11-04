package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

public class Heading {
public static double calculateTh(Pose currentPose, Pose targetPose) {
        double dx = targetPose.x() - currentPose.x();
        double dy = targetPose.y() - currentPose.y();
        return Math.atan2(dy, dx);
    }
}