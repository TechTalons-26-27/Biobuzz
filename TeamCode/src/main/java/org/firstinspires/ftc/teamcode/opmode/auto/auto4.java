package org.firstinspires.ftc.teamcode.opmode.auto;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.config.pedro.Constants;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class auto4 extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(57.7048, 8, 90);
    private final Pose path1 = poseFactory.of(57.989, 29.749, 90);
    private final Pose point2 = poseFactory.of(11.5352, 46.7943, 176.8535);
    private final Pose point2Control1 = poseFactory.of(58.3691, 41.3258, 0);
    private final Pose point2Control2 = poseFactory.of(25.9217, 46.0331, 0);
    private final Pose point3 = poseFactory.of(23.7808, 46.2145, 177.2896);
    private final Pose point4 = poseFactory.of(58.7099, 114.6326, -89.3543);
    private final Pose point4Control1 = poseFactory.of(0.9991, 55.2477, 0);
    private final Pose point4Control2 = poseFactory.of(29.9503, 136.2063, 0);
    private final Pose point4Control3 = poseFactory.of(58.9116, 135.6068, 0);
    private final Pose point5 = poseFactory.of(47.5856, 131.0497, 90.6843);
    private final Pose point5Control1 = poseFactory.of(58.1335, 103.9807, 0);
    private final Pose point5Control2 = poseFactory.of(47.733, 110.0359, 0);
    private final Pose point6 = poseFactory.of(56.2634, 117.0037, -94.7756);
    private final Pose point6Control1 = poseFactory.of(47.2182, 110.3002, 0);
    private final Pose point6Control2 = poseFactory.of(55.3352, 104.8637, 0);
    private final Pose point7 = poseFactory.of(12.7256, 108.3619, -1.7134);
    private final Pose point7Control1 = poseFactory.of(56.4659, 134.3462, 0);
    private final Pose point7Control2 = poseFactory.of(24.954, 107.7201, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5()),
                follow(follower, path6()),
                follow(follower, path7())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        waitForStart();
        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            telemetry.addData("x", follower.pose().x());
            telemetry.addData("y", follower.pose().y());
            telemetry.addData("heading", follower.pose().heading());

            if (follower.currentPath() != null) {
                telemetry.addData("Current path distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Path path1() {
        return line(start, path1).linear(start, path1);
    }

    public Path path2() {
        return curve(path1, point2Control1, point2Control2, point2).tangent();
    }

    public Path path3() {
        return line(point2, point3).reverseTangent();
    }

    public Path path4() {
        return curve(point3, point4Control1, point4Control2, point4Control3, point4).tangent();
    }

    public Path path5() {
        return curve(point4, point5Control1, point5Control2, point5).tangent();
    }

    public Path path6() {
        return curve(point5, point6Control1, point6Control2, point6).reverseTangent();
    }

    public Path path7() {
        return curve(point6, point7Control1, point7Control2, point7).reverseTangent();
    }
}