
package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public class Drive {

    private final Follower follower;

    public Drive(HardwareMap hardwareMap, Pose startPose) {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
    }

    public void run(double forward, double lateral, double rotate) {
        DrivePowers powers = ManualDrive.fieldCentric(
                forward,
                lateral,
                rotate,
                follower.pose().heading()
        );

        follower.manual(powers);
        follower.update();
    }

    public Pose getOdometryPose() {
        return follower.pose();
    }

    public void stop() {
        follower.stop();
        follower.update();
    }
}
