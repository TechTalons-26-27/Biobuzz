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

@Autonomous(name = "auto2", group = "Autonomous")
public class auto2 extends LinearOpMode {

    private Follower follower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(59.3028, 132.8351, 90);
    private final Pose path1Start = poseFactory.of(59.3028, 132.8351, -90);
    private final Pose path1 = poseFactory.of(59.1798, 109.7708, -90);
    private final Pose point2 = poseFactory.of(47.1011, 130.264, 88.4289);
    private final Pose point2Control1 = poseFactory.of(56.8708, 99.9079, 0);
    private final Pose point2Control2 = poseFactory.of(46.1337, 98.9124, 0);
    private final Pose point3Start = poseFactory.of(47.1011, 130.264, 90);
    private final Pose point3 = poseFactory.of(47.0014, 115.9687, 90);
    private final Pose point4 = poseFactory.of(58.9848, 109.9407, -97.1106);
    private final Pose point4Control1 = poseFactory.of(48.0135, 129.9461, 0);
    private final Pose point4Control2 = poseFactory.of(61.4697, 128.7663, 0);
    private final Pose point5 = poseFactory.of(8.9539, 107.4517, 179.4141);
    private final Pose point5Control1 = poseFactory.of(54.1146, 84.7831, 0);
    private final Pose point5Control2 = poseFactory.of(32.1978, 107.4382, 0);

    // Autonomous routine
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3()),
                follow(follower, path4()),
                follow(follower, path5())
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
        return line(path1Start, path1).linear(path1Start, path1);
    }

    public Path path2() {
        return curve(path1, point2Control1, point2Control2, point2).tangent();
    }

    public Path path3() {
        return line(point3Start, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return curve(point3, point4Control1, point4Control2, point4).tangent();
    }

    public Path path5() {
        return curve(point4, point5Control1, point5Control2, point5).tangent();
    }
}