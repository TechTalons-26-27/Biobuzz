package org.firstinspires.ftc.teamcode.auto;

import static com.pedropathing.api.Paths.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "auto3", group = "Autonomous")
public class auto3 extends LinearOpMode {

    private Follower follower;

    private DcMotor stages;
    private DcMotor outtake;
    private DcMotor intake;
    private Servo finger;

    private double open;

    private double block;

    private double intakePower;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose path1 = poseFactory.of(56.2308, 26.3051, 90);
    private final Pose point2 = poseFactory.of(26.5946, 8.5987, 178.8806);
    private final Pose point2Control1 = poseFactory.of(57.2178, 46.1517, 0);
    private final Pose point2Control2 = poseFactory.of(21.146, 37.6126, 0);
    private final Pose point2Control3 = poseFactory.of(43.3018, 7.8303, 0);
    private final Pose point3 = poseFactory.of(6.2928, 8.602, 179.9908);
    private final Pose point4 = poseFactory.of(27.221, 8.6664, -179.8236);
    private final Pose point5 = poseFactory.of(56.2219, 123.0897, -69.5174);
    private final Pose point5Control1 = poseFactory.of(57.3768, 4.7586, 0);
    private final Pose point5Control2 = poseFactory.of(21.7635, 58.9902, 0);
    private final Pose point5Control3 = poseFactory.of(20.062, 105.0914, 0);
    private final Pose point5Control4 = poseFactory.of(62.9927, 107.3148, 0);
    private final Pose point6 = poseFactory.of(30.7651, 125.4698, -118.0795);
    private final Pose point6Control1 = poseFactory.of(55.9984, 134.1615, 0);
    private final Pose point6Control2 = poseFactory.of(37.9192, 120.3148, 0);
    private final Pose point6Control3 = poseFactory.of(26.0595, 116.9347, 0);
    private final Pose point7 = poseFactory.of(36.8377, 129.3931, 32.8653);
    private final Pose point8 = poseFactory.of(55.9217, 122.7757, 35.1741);
    private final Pose point8Control1 = poseFactory.of(49.5367, 118.1746, 0);
    private final Pose point9 = poseFactory.of(9.3695, 60.699, -93.2619);
    private final Pose point9Control1 = poseFactory.of(55.7643, 107.5432, 0);
    private final Pose point9Control2 = poseFactory.of(29.9878, 78.0767, 0);
    private final Pose point9Control3 = poseFactory.of(9.9258, 75.7562, 0);
    private final Pose point10 = poseFactory.of(56.3777, 26.5269, 87.6979);
    private final Pose point10Control1 = poseFactory.of(7.0579, 46.1476, 0);
    private final Pose point10Control2 = poseFactory.of(22.137, 15.6085, 0);
    private final Pose point10Control3 = poseFactory.of(55.9886, 3.9812, 0);
    private final Pose point11 = poseFactory.of(8.9144, 101.77, 88.9372);
    private final Pose point11Control1 = poseFactory.of(55.7667, 54.5392, 0);
    private final Pose point11Control2 = poseFactory.of(8.4502, 39.8401, 0);
    private final Pose point11Control3 = poseFactory.of(8.3695, 73.2235, 0);

    private Command runintake;
    private Command runouttake;

    private Command rampupouttake;
    private Command unintake;
    private Command hold;
    private Command timedActionSequence;
    private Command WarmUp;
    public Command autoRoutine() {
        return sequential(
                parallel(
                        follow(follower, path1()),
                        rampupouttake
                ),
                runouttake,
                follow(follower, path2()),
                parallel(
                        runintake,
                        follow(follower, path3())
                ),
                parallel(
                        runintake,
                        follow(follower, path4())
                ),
                parallel(
                        rampupouttake,
                        follow(follower, path5())
                ),

                follow(follower, path6()),
                follow(follower, path7()),
                follow(follower, path8()),
                follow(follower, path9()),
                follow(follower, path10()),
                follow(follower, path11())
        );
    }

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        outtake = hardwareMap.get(DcMotor.class, "outtake");
        intake = hardwareMap.get(DcMotor.class, "intake");
        stages = hardwareMap.get(DcMotor.class, "stages");
        finger = hardwareMap.get(Servo.class, "finger");
        stages.setDirection(DcMotor.Direction.FORWARD);

        runintake = Command.build()
                .setExecute(() -> {intake.setPower(intakePower); stages.setPower(intakePower);})
                .setDone(() -> intake.getCurrentPosition() >1000)
                .setEnd(endCondition -> {intake.setPower(0); stages.setPower(0);})
                .requiring(intake);

        Command extendOuttake = Command.build()
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


        rampupouttake = Command.build()
                .setExecute(() ->{ intake.setPower(-intakePower); outtake.setPower(-0.7); stages.setPower(-intakePower); finger.setPosition(block);})
                .setDone(() -> outtake.getCurrentPosition() >1000)
                .setEnd(endCondition ->{ intake.setPower(0); outtake.setPower(0); stages.setPower(0); finger.setPosition(50);})
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

    public Path path3() {
        return line(point2, point3).tangent();
    }

    public Path path4() {
        return line(point3, point4).reverseTangent();
    }

    public Path path5() {
        return curve(point4, point5Control1, point5Control2, point5Control3, point5Control4, point5).reverseTangent();
    }

    public Path path6() {
        return curve(point5, point6Control1, point6Control2, point6Control3, point6).reverseTangent();
    }

    public Path path7() {
        return line(point6, point7).tangent();
    }

    public Path path8() {
        return curve(point7, point8Control1, point8).tangent();
    }

    public Path path9() {
        return curve(point8, point9Control1, point9Control2, point9Control3, point9).tangent();
    }

    public Path path10() {
        return curve(point9, point10Control1, point10Control2, point10Control3, point10).tangent();
    }

    public Path path11() {
        return curve(point10, point11Control1, point11Control2, point11Control3, point11).tangent();
    }
}
