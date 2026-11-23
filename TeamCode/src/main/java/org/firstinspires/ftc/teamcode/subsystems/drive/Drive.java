
package org.firstinspires.ftc.teamcode.subsystems.drive;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.Heading;

public class Drive {

    private Follower follower;

    double kP = 0;
    double error = 0;
    double lastError = 0;
    double angleTolerance = 0.2;
    double kD = 0.5;
    double curTime = 0;
    double lastTime = 0;


    public Drive(HardwareMap hardwareMap, Pose startPose) {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
    }

    public Pose getPose() {
        return follower.pose();
    }

    public void run(double forward, double lateral, double rotate) {
        DrivePowers powers = ManualDrive.fieldCentric(
                forward,
                lateral,
                rotate,
                follower.pose().heading()
        );

        ManualDrive.driveOrHold(follower, powers);
        follower.update();
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("h", follower.pose().heading());
        telemetry.addData("error", error);
    }

    public void start(double curTime) {
        //resetRuntime()
        this.curTime = curTime;
    }

    public void stop() {
        follower.stop();
        follower.update();
    }

    public void updatePose(Pose updatedPose) {
        follower.setPose(updatedPose);
    }

    public double track(Pose currentPose, Pose targetPose, double curTime) {
        double ch = currentPose.heading();
        double th = Heading.calculateTh(currentPose, targetPose);

        error = Math.atan2(Math.sin(th - ch), Math.cos(th - ch));

        double dT = curTime - lastTime;
        double dTerm = (dT > 0) ? ((error - lastError) / dT) * kD : 0;

        lastError = error;
        lastTime = curTime;
        this.curTime = curTime;

        if (Math.abs(error) < angleTolerance) {
            return 0;
        }
        return Range.clip(error * kP + dTerm, -0.7, 0.7);
    }
}