package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.vision.Limelight;
import org.firstinspires.ftc.teamcode.util.KalmanFilter;

public class FusionLocalizer {
    private final Follower follower;
    private final ElapsedTime timer;
    private final KalmanFilter kalmanFilter;
    private final Limelight limelight;

    public FusionLocalizer(HardwareMap hardwareMap, Pose startPose) {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        follower.update();

        limelight = new Limelight(hardwareMap,1);

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

        Vector2D velocity = follower.velocity().toVector2D();

        kalmanFilter.predict(velocity, dt, follower.pose().heading());

        //TODO: TUNE LIMELIGHT VARIANCE
        if (limelight.isValid()) {
            kalmanFilter.update(limelight.getNewFieldPose(), 4.0, follower.pose().heading());
        }
    }

    public Pose getPose() {
        return kalmanFilter.getPose();
    }
}
