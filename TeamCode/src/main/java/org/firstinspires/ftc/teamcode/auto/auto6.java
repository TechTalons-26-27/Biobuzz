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
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "auto6", group = "Autonomous")
public class auto6 extends LinearOpMode {

    private Follower follower;

    private DcMotor stage1;
    private DcMotor outtake;
    private DcMotor intake;
    private DcMotor stage2;
    private DcMotor finger;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(56.4617, 29.075, 90);
    private final Pose point2 = poseFactory.of(29.6044, 54.376, 136.7091);
    private final Pose point3 = poseFactory.of(11.7455, 99.314, 111.6735);

    private Command runintake;
    private Command runouttake;
    private Command unintake;
    private Command hold;
    public Command autoRoutine() {
        return sequential(
                follow(follower, path1()),
                follow(follower, path2()),
                follow(follower, path3())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        stage1 = hardwareMap.get(DcMotor.class, "stage1");
        outtake = hardwareMap.get(DcMotor.class, "outtake");
        intake = hardwareMap.get(DcMotor.class, "intake");
        stage2 = hardwareMap.get(DcMotor.class, "stage2");
        finger = hardwareMap.get(DcMotor.class, "finger");
        stage1.setDirection(DcMotor.Direction.FORWARD);

        runintake = Command.build()
                .setExecute(() -> {intake.setPower(0.7); stage1.setPower(0.7); stage2.setPower(0.7);})
                .setDone(() -> intake.getCurrentPosition() >1000)
                .setEnd(endCondition -> {intake.setPower(0); stage1.setPower(0); stage2.setPower(0); })
                .requiring(intake);

        runouttake = Command.build()
                .setExecute(() ->{ intake.setPower(-0.7); outtake.setPower(-0.7); stage1.setPower(-0.7); stage2.setPower(-0.7); finger.setPower(0.5);})
                .setDone(() -> outtake.getCurrentPosition() >1000)
                .setEnd(endCondition ->{ intake.setPower(0); outtake.setPower(0); stage1.setPower(0); stage2.setPower(0); finger.setPower(0);})
                .requiring(outtake);
        Command WarmUp = sequential(
                instant(() -> {
                    intake.setPower(-1.0);
                    outtake.setPower(-1.0);
                    stage1.setPower(-1.0);
                    stage2.setPower(-1.0);
                    finger.setPower(1.0);
                }),

                waitMs(1000),

                instant(() -> {
                    intake.setPower(0);
                    outtake.setPower(0);
                    stage1.setPower(0);
                    stage2.setPower(0);
                    finger.setPower(0);
                })
        );
        Command timedActionSequence = sequential(
                instant(() -> {
                    intake.setPower(-0.7);
                    outtake.setPower(-0.7);
                    stage1.setPower(-0.7);
                    stage2.setPower(-0.7);
                    finger.setPower(0.5);
                }),

                waitMs(1000),

                instant(() -> {
                    intake.setPower(0);
                    outtake.setPower(0);
                    stage1.setPower(0);
                    stage2.setPower(0);
                    finger.setPower(0);
                })
        );
        unintake = Command.build()
                .setExecute(() -> {intake.setPower(-0.7); stage1.setPower(-0.7); stage2.setPower(-0.7);})
                .setDone(() -> intake.getCurrentPosition() >1000)
                .setEnd(endCondition -> {intake.setPower(0); stage1.setPower(0); stage2.setPower(0); })
                .requiring(intake);
        hold = Command.build()
                .setExecute(() -> {intake.setPower(0.3); stage1.setPower(0.3); stage2.setPower(-0.3);})
                .setDone(() -> intake.getCurrentPosition() >1000)
                .setEnd(endCondition -> {intake.setPower(0); stage1.setPower(0); stage2.setPower(0); })
                .requiring(intake);

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
        return line(path1, point2).tangent();
    }

    public Path path3() {
        return line(point2, point3).tangent();
    }
}
