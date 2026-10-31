
package org.firstinspires.ftc.teamcode.subsystems.drive;

import static com.pedropathing.api.Paths.line;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.util.Heading;
import org.firstinspires.ftc.teamcode.util.PoseStorage;

public class Drive {

    private Follower follower;

    double kP = -0.0200;
    double error = 0;
    double lastError = 0;
    double angleTolerance = 0.2;
    double kD = -0.0005;
    double curTime = 0;
    double lastTime = 0;

    // --------------------------- Drive ---------------------------

    public Drive(HardwareMap hardwareMap, Pose startPose) {
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
    }

    public Pose getOdometryPose() {
        return follower.pose();
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

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("x", follower.pose().x());
        telemetry.addData("y", follower.pose().y());
        telemetry.addData("h", follower.pose().heading());
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
        double rotate = 0;

        double ch = currentPose.heading();
        double th = Math.toDegrees(Heading.calculateTh(currentPose, targetPose));

        double error = th - ch;
        error = Math.atan2(Math.sin(error), Math.cos(error));

        if (Math.abs(error) < angleTolerance) {
            rotate = 0;
        } else {
            double pTerm = error * kP;
            this.curTime = curTime;

            double dT = curTime - lastTime;

            double dTerm = 0;

            if (dT != 0) {
                dTerm = ((error - lastError) / dT) * kD;
            }

            rotate = Range.clip(pTerm + dTerm, -0.4, 0.4);

            lastError = error;
            lastTime = curTime;
        }
        return rotate;
    }
}