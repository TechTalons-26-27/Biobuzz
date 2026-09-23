package org.firstinspires.ftc.teamcode.opmode.auto;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.config.pedro.Constants;

@Autonomous(name = "AutoPath", group = "Autonomous")
public class autoPath3 extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(58.2505, 8.2813, 90);
    private final Pose path1 = poseFactory.of(58.2505, 32.3429, 90);
    private final Pose point2 = poseFactory.of(41.7187, 25.7008, -102.21);
    private final Pose point2Control1 = poseFactory.of(57.1481, 45.8688, 0);
    private final Pose point2Control2 = poseFactory.of(43.5219, 34.5527, 0);
    private final Pose point3 = poseFactory.of(8.5527, 8.8917, 179.6949);
    private final Pose point3Control1 = poseFactory.of(38.3678, 8.6471, 0);
    private final Pose point4Start = poseFactory.of(8.5527, 8.8917, -178);
    private final Pose point4 = poseFactory.of(20.4026, 14.2445, 90);
    private final Pose point5 = poseFactory.of(11.1133, 105.5626, 175.9548);
    private final Pose point5Control1 = poseFactory.of(20.2674, 35.6511, 0);
    private final Pose point5Control2 = poseFactory.of(30.6083, 104.8867, 0);
    private final Pose point6Start = poseFactory.of(11.1133, 105.5626, 180);
    private final Pose point6 = poseFactory.of(58.0308, 109.8559, -90);
    private final Pose point7 = poseFactory.of(11.7724, 109.4702, -90);

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
        return curve(
                path1,
                point2Control1,
                point2Control2,
                point2
        );
    }

    public Path path3() {
        return curve(
                point2,
                point3Control1,
                point3
        );
    }

    public Path path4() {
        return line(point4Start, point4).linear(point4Start, point4);
    }

    public Path path5() {
        return curve(
                point4,
                point5Control1,
                point5Control2,
                point5
        );
    }

    public Path path6() {
        return line(point6Start, point6).linear(point6Start, point6);
    }

    public Path path7() {
        return line(point6, point7).linear(point6, point7);
    }
}