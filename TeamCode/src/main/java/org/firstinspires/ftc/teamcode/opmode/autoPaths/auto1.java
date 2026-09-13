package org.firstinspires.ftc.teamcode.opmode.autoPaths; // make sure this aligns with class location

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
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.teamcode.config.pedropathing.Constants;

@Autonomous(name = "3rd BioBuzz Auto Path ", group = "Examples")
public class auto1 extends LinearOpMode {

    private Follower follower;

    //defining our PathChains
    private PathChain mainPath1, mainPath2;

    private DcMotor intake;
    private DcMotor outtake;

    double power = 0;
    public void buildPaths() {

        mainPath1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(57.5899, 8.636),
                                new Pose(57.9079, 30.9124)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();

        mainPath2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(57.9079, 30.9124),
                                new Pose(58.7191, 47.8978),
                                new Pose(3.8472, 18.3303),
                                new Pose(45.2899, 104.4843),
                                new Pose(10.2157, 106.7337)
                        )
                )
                .setTangentHeadingInterpolation()
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
                follow(follower, mainPath2,  true)


        );

    }



    @Override
    public void runOpMode() {
        //These will run when the OpMode is initiated

        Scheduler.reset();
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(new Pose(57.5899, 8.636, Math.toRadians(90)));
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
