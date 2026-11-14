package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Basic TeleOp (Robot Centric)")
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


        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);



        //outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        waitForStart();


        while (opModeIsActive()) {

            while (opModeIsActive()) {

                // Joystick inputs
                double y = -gamepad1.left_stick_y;  // Forward / backward
                double x = gamepad1.left_stick_x;   // Strafe left / right
                double rx = gamepad1.right_stick_x; // Rotation

                // Mecanum calculations
                double frontLeftPower = y + x + rx;
                double backLeftPower = y - x + rx;
                double frontRightPower = y - x - rx;
                double backRightPower = y + x - rx;

                // Normalize powers
                double max = Math.max(1.0,
                        Math.max(Math.abs(frontLeftPower),
                                Math.max(Math.abs(backLeftPower),
                                        Math.max(Math.abs(frontRightPower),
                                                Math.abs(backRightPower)))));

                frontLeftPower /= max;
                backLeftPower /= max;
                frontRightPower /= max;
                backRightPower /= max;

                // Set motor powers
                frontLeft.setPower(frontLeftPower);
                backLeft.setPower(backLeftPower);
                frontRight.setPower(frontRightPower);
                backRight.setPower(backRightPower);

                telemetry.update();
            }

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