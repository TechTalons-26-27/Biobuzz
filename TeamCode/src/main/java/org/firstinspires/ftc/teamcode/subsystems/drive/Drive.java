package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Vector2D;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.KalmanFilter;

public class Drive {
    private final Follower follower;

    private ElapsedTime timer;

    private KalmanFilter kalmanFilter;

    public Drive(HardwareMap hardwareMap, Pose startPose) {
        follower = Constants.create(hardwareMap);

        timer = new ElapsedTime();

        kalmanFilter = new KalmanFilter(startPose, 1, 1);
    }

    public void start() {
        timer.reset();
    }

    public void run(double forward, double lateral, double rotate) {
        follower.update();

        double dt = timer.seconds();
        timer.reset();

        Vector2D velocity = follower.velocity().toVector2D();

        kalmanFilter.predict(velocity, dt);

        Pose correctedPose = new Pose(kalmanFilter.getPose().x(), kalmanFilter.getPose().y(), follower.pose().heading());
        follower.setPose(correctedPose);


        DrivePowers powers = ManualDrive.fieldCentric(
                    forward,
                    lateral,
                    rotate,
                    follower.pose().heading()
            );
        ManualDrive.driveOrHold(follower, powers);

    }
}
