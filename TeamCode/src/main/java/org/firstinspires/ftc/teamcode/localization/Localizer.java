package org.firstinspires.ftc.teamcode.localization;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;

import org.firstinspires.ftc.teamcode.subsystems.vision.Limelight;

public class Localizer {
    private static final boolean ENABLE_VISION = false;

    private final Follower follower;
    private final ElapsedTime timer;
    private final KalmanFilter kalmanFilter;
    private Pose lastOdomPose;
    private final Limelight limelight;

    public Localizer(HardwareMap hardwareMap, Pose startPose) {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();
        lastOdomPose = follower.pose();

        limelight = ENABLE_VISION ? new Limelight(hardwareMap, 1) : null;

        timer = new ElapsedTime();

        //TODO: TUNE INITIAL VARIANCE
        kalmanFilter = new KalmanFilter(startPose, 1.0, 1.0);
    }

    public void start() {
        timer.reset();
    }

    public void run() {
        follower.update();

        double dt = timer.seconds();
        timer.reset();

        Pose odomPose = follower.pose();
        kalmanFilter.predict(
                odomPose.x() - lastOdomPose.x(),
                odomPose.y() - lastOdomPose.y(),
                dt,
                odomPose.heading()
        );
        lastOdomPose = odomPose;

        //TODO: TUNE LIMELIGHT VARIANCE
        if (limelight != null) {
            Pose visionPose = limelight.getNewFieldPose();
            if (visionPose != null) {
                kalmanFilter.update(visionPose, 4.0, odomPose.heading());
            }
        }
    }

    public Pose getPose() {
        return kalmanFilter.getPose();
    }
}
