/*
package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.deadline;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "auto5", group = "Autonomous")
public class auto5 extends LinearOpMode {

    private Follower follower;

    private DcMotor stages;
    private DcMotor outtake;
    private DcMotor intake;

    private Servo finger;

    double block = 0;
    double open = 0;

    double intakePower = 0;

    double outtakePower = 0;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(55.5501, 26.5517, 90);
    private final Pose point2 = poseFactory.of(12.0221, 45.8338, 173.5623);
    private final Pose point2Control1 = poseFactory.of(55.6009, 40.9166, 0);
    private final Pose point3 = poseFactory.of(55.5782, 117.6133, -89.2302);
    private final Pose point3Control1 = poseFactory.of(3.101, 68.991, 0);
    private final Pose point3Control2 = poseFactory.of(13.5322, 103.9722, 0);
    private final Pose point3Control3 = poseFactory.of(55.9145, 92.5857, 0);
    private final Pose point4 = poseFactory.of(51.3157, 128.9316, 110.6366);
    private final Pose point5 = poseFactory.of(54.8658, 117.8798, -87.8029);
    private final Pose point5Control1 = poseFactory.of(54.6598, 123.2489, 0);
    private final Pose point6 = poseFactory.of(55.6341, 26.9021, 89.9607);
    private final Pose point6Control1 = poseFactory.of(7.2707, 87.0804, 0);
    private final Pose point6Control2 = poseFactory.of(0, 38.8406, 0);
    private final Pose point6Control3 = poseFactory.of(55.6396, 34.9757, 0);
    private final Pose point7 = poseFactory.of(11.2499, 97.619, 93.6919);
    private final Pose point7Control1 = poseFactory.of(60.2678, 37.3277, 0);
    private final Pose point7Control2 = poseFactory.of(3.104, 34.8364, 0);
    private final Pose point7Control3 = poseFactory.of(12.3057, 81.2568, 0);

    private Command runintake;
    private Command runouttake;
    private Command unintake;
    private Command hold;
    private Command timedActionSequence;
    private Command WarmUp;

    private Command rampupouttake;
    public Command autoRoutine() {
        return sequential(
                deadline(
                    follow(follower, path1()),
                    rampupouttake
                        ),
                runouttake,
                follow(follower, path2()),
                runintake,
                follow(follower, path3()),
                parallel(
                    follow(follower, path4()),
                    rampupouttake
                        ),
                runouttake,
                deadline(
                    follow(follower, path5()),
                    runintake
                        ),
                runouttake,
                deadline(
                    follow(follower, path6()),
                    runintake
                        ),
                runouttake,
                follow(follower, path7())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        stages = hardwareMap.get(DcMotor.class, "stage1");
        outtake = hardwareMap.get(DcMotor.class, "outtake");
        intake = hardwareMap.get(DcMotor.class, "intake");
        finger = hardwareMap.get(Servo.class, "finger");
        stages.setDirection(DcMotor.Direction.FORWARD);

        runintake = Command.build()
                .setExecute(() -> {intake.setPower(intakePower); stages.setPower(intakePower); finger.setPosition(block); })
                .setDone(() -> intake.getCurrentPosition() >1000)
                .setEnd(endCondition -> {intake.setPower(0); stages.setPower(0); })
                .requiring(intake);

        rampupouttake = Command.build()
                .setExecute(() ->{ outtake.setPower(outtakePower); })
                .setDone(() -> outtake.getCurrentPosition() >1000)
                .setEnd(endCondition ->{ outtake.setPower(0); })
                .requiring(outtake);

        runouttake = Command.build()
                .setStart(() -> {
                    outtake.setTargetPosition(1000);
                    outtake.setMode(
                            DcMotor.RunMode.RUN_TO_POSITION
                    );
                    intake.setPower(0.7);
                    stages.setPower(0.7);
                    outtake.setPower(0.7);
                    finger.setPosition(open);
                })
                .setDone(() ->
                        Math.abs(
                                outtake.getTargetPosition()
                                        - outtake.getCurrentPosition()
                        ) < 15
                )
                .setEnd(endCondition ->{
                            outtake.setPower(0);
                            intake.setPower(0);
                            stages.setPower(0);
                            finger.setPosition(block);
                        }
                )
                .requiring(outtake);

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
        return curve(path1, point2Control1, point2).tangent();
    }

    public Path path3() {
        return curve(point2, point3Control1, point3Control2, point3Control3, point3).reverseTangent();
    }

    public Path path4() {
        return line(point3, point4).tangent();
    }

    public Path path5() {
        return curve(point4, point5Control1, point5).tangent();
    }

    public Path path6() {
        return curve(point5, point6Control1, point6Control2, point6Control3, point6).reverseTangent();
    }

    public Path path7() {
        return curve(point6, point7Control1, point7Control2, point7Control3, point7).tangent();
    }
}
*/
