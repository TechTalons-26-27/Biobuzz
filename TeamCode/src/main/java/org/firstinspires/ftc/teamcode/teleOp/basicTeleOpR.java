package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Basic TeleOp (Field Centric)")
public class basicTeleOpR extends LinearOpMode {

    //Pedro Constants

    //Motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;


    @Override
    public void runOpMode() {

        // Hardware Maps

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        //intake = hardwareMap.get(DcMotor.class, "intake");
        //stage2 = hardwareMap.get(DcMotor.class, "stage2");
        //outtake = hardwareMap.get(DcMotorEx.class, "outtake");


        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);



        //outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        waitForStart();


        while (opModeIsActive()) {

            double rx = gamepad1.right_stick_x;


            // Mecanum calculations
            double frontLeftPower = rx;
            double backLeftPower = rx;
            double frontRightPower = rx;
            double backRightPower = rx;


            // Normalize
            double max = Math.max(
                    1.0,
                    Math.max(
                            Math.abs(frontLeftPower),
                            Math.max(
                                    Math.abs(backLeftPower),
                                    Math.max(
                                            Math.abs(frontRightPower),
                                            Math.abs(backRightPower)
                                    )
                            )
                    )
            );


            frontLeft.setPower(frontLeftPower / max);
            backLeft.setPower(backLeftPower / max);
            frontRight.setPower(frontRightPower / max);
            backRight.setPower(backRightPower / max);

/*

            if (gamepad2.a) {
                intake.setPower(1.0);
            } else {
                intake.setPower(0);
            }



            if (gamepad2.b) {
                stage2.setPower(-1.0);
            } else {
                stage2.setPower(1.0);
            }



            if (gamepad2.right_trigger > 0.05) {

                // CHANGE THIS TARGET VELOCITY
                outtakeTargetVelocity = 2000;

                outtake.setVelocity(
                        outtakeTargetVelocity
                );

            } else {
                outtakeTargetVelocity = 0;
                outtake.setVelocity(0);
            }

*/

            // Telemetry

            /*
            telemetry.addData(
                    "Outtake Velocity"
                    //outtake.getVelocity()
            );

            telemetry.addData(
                    "Outtake Target",
                    //outtakeTargetVelocity
            );
            */


            telemetry.update();
        }
    }
}