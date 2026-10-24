package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.behaviors.BlockedBehavior;
import com.pedropathing.ivy.behaviors.ConflictBehavior;
import com.pedropathing.ivy.behaviors.InterruptedBehavior;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "auto1", group = "Autonomous")
public class auto1 extends LinearOpMode {

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

    private final Pose start = poseFactory.of(56.4617, 8, 90);
    private final Pose path1 = poseFactory.of(56.6925, 21.9192, 90);
    private final Pose point2 = poseFactory.of(10.1436, 101.8499, 90.0475);
    private final Pose point2Control1 = poseFactory.of(57.4527, 42.0008, 0);
    private final Pose point2Control2 = poseFactory.of(12.2202, 25.0653, 0);
    private final Pose point2Control3 = poseFactory.of(10.1297, 75.9331, 0);

    private Command runintake;
    private Command runouttake;
    private Command unintake;
    private Command hold;
    private Command timedActionSequence;
    private Command WarmUp;

    private Command rampupouttake;



    public Command autoRoutine() {
        return sequential(
                parallel(
                    follow(follower, path1()),
                    rampupouttake
                ),
                runouttake,
                follow(follower, path2())
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
        return curve(path1, point2Control1, point2Control2, point2Control3, point2).tangent();
    }
}
