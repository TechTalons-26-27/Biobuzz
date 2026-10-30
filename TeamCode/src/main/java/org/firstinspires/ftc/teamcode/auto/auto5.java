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

@Autonomous(name = "AutoPath", group = "Autonomous")
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

    private final Pose start = poseFactory.of(58.5463, 9.0913, 90);
    private final Pose path1 = poseFactory.of(58.5643, 34.1019, 90);
    private final Pose point2 = poseFactory.of(13.2059, 47.0116, 179.4569);
    private final Pose point2Control1 = poseFactory.of(53.177, 49.6467, 0);
    private final Pose point2Control2 = poseFactory.of(41.3638, 46.7147, 0);
    private final Pose point3Start = poseFactory.of(13.2059, 47.0116, 180);
    private final Pose point3 = poseFactory.of(25.6247, 46.9846, 180);
    private final Pose point4 = poseFactory.of(58.9871, 107.8342, -91.6451);
    private final Pose point4Control1 = poseFactory.of(22.3548, 118.5707, 0);
    private final Pose point4Control2 = poseFactory.of(60.2674, 139.3805, 0);
    private final Pose point5 = poseFactory.of(58.6388, 33.7134, 87.5897);
    private final Pose point5Control1 = poseFactory.of(55.4409, 94.8316, 0);
    private final Pose point5Control2 = poseFactory.of(-1.3907, 105.563, 0);
    private final Pose point5Control3 = poseFactory.of(58.1748, -0.0206, 0);
    private final Pose point6 = poseFactory.of(58.9897, 107.6568, -88.2141);
    private final Pose point6Control1 = poseFactory.of(4.9833, 92.374, 0);
    private final Pose point6Control2 = poseFactory.of(42.4486, 105.5578, 0);
    private final Pose point6Control3 = poseFactory.of(58.5437, 130.2095, 0);
    private final Pose point7 = poseFactory.of(10.4306, 107, 178.7954);
    private final Pose point7Control1 = poseFactory.of(57.5488, 92, 0);
    private final Pose point7Control2 = poseFactory.of(21.0694, 106.9177, 0);

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
        return curve(path1, point2Control1, point2Control2, point2).tangent();
    }

    public Path path3() {
        return line(point3Start, point3).linear(point3Start, point3);
    }

    public Path path4() {
        return curve(point3, point4Control1, point4Control2, point4).tangent();
    }

    public Path path5() {
        return curve(point4, point5Control1, point5Control2, point5Control3, point5).tangent();
    }

    public Path path6() {
        return curve(point5, point6Control1, point6Control2, point6Control3, point6).tangent();
    }

    public Path path7() {
        return curve(point6, point7Control1, point7Control2, point7).tangent();
    }
}
