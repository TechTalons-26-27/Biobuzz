package org.firstinspires.ftc.teamcode.opmode.auto; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import static com.pedropathing.ivy.Scheduler.*;
import static com.pedropathing.ivy.pedro.PedroCommands.*;
import static com.pedropathing.ivy.groups.Groups.*;

import org.firstinspires.ftc.teamcode.config.pedropathing.Constants;

@Autonomous(name = "3rd BioBuzz Auto Path ", group = "Examples")
public class autoPath3 extends LinearOpMode {

    private Follower follower;

    //defining our PathChains
    private PathChain mainPath1, mainPath2, mainPath3, mainPath4, mainPath5, mainPath6, mainPath7;

    private DcMotor intake;
    private DcMotor outtake;

    double power = 0;
    public void buildPaths() {

        mainPath1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(58.2505, 8.000),
                                new Pose(58.2505, 32.3429)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        mainPath2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(58.2505, 32.3429),
                                new Pose(54.8976, 46.1501),
                                new Pose(45.3757, 25.4195)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        mainPath3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(45.3757, 25.4195),
                                new Pose(38.3678, 8.6471),
                                new Pose(7.9901, 8.329)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        mainPath4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(7.9901, 8.329),
                                new Pose(20.9652, 8.337)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-178), Math.toRadians(90))
                .build();

        mainPath5 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(20.9652, 8.337),
                                new Pose(20.2674, 35.6511),
                                new Pose(30.6083, 104.8867),
                                new Pose(11.1133, 105.5626)
                                )
                )
                .setTangentHeadingInterpolation()
                .build();

        mainPath5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(11.1133, 105.5626),
                                new Pose(58.0308, 109.8559)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(-90))
                .build();

        mainPath5 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(58.0308, 109.8559),
                                new Pose(11.7724, 109.4702)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();
    }
    private Command intakeIn;
    private Command outtakeOut;

    public Command autoRoutine() {
        return sequential(
                deadline(
                        sequential(
                                follow(follower, mainPath1,  true)
                        ),
                        outtakeOut

                ),
                deadline(
                        sequential(
                                follow(follower, mainPath2),
                                follow(follower, mainPath3, true),
                                follow(follower, mainPath4, true),
                                follow(follower, mainPath5, true)


                        ),
                        intakeIn
                ),
                deadline(
                        sequential(
                                follow(follower, mainPath6),
                                follow(follower, mainPath7,  true)

                        ),
                        outtakeOut

                )

        );

    }



    @Override
    public void runOpMode() {
        //These will run when the OpMode is initiated

        Scheduler.reset();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(new Pose(58.2505, 8.2813, Math.toRadians(90)));
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.FORWARD);

        outtake = hardwareMap.get(DcMotor.class, "outtake");
        outtake.setDirection(DcMotor.Direction.FORWARD);

        intakeIn = Command.build()
                .setExecute(() -> intake.setPower(0.7))
                .setDone(() -> intake.getCurrentPosition() >1000)
                .setEnd(endCondition -> intake.setPower(0))
                .requiring(intake);

        outtakeOut = Command.build()
                .setExecute(() -> outtake.setPower(0.7))
                .setDone(() -> outtake.getCurrentPosition() >1000)
                .setEnd(endCondition -> outtake.setPower(0))
                .requiring(outtake);



        waitForStart();
        //We schedule all our commands when we start the OpMode
        schedule(autoRoutine());
        while (opModeIsActive()) {
            //Update the follower and execute the scheduler every loop
            follower.update();
            Scheduler.execute();

            // Feedback to Driver Hub for debugging
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }



}