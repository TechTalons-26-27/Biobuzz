package org.firstinspires.ftc.teamcode.opmode.teleOp;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Basic TeleOp")
public class basicTeleOp extends LinearOpMode {

    //Pedro Constants

    //Motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private DcMotor intake;
    private DcMotor stage2;
    private DcMotorEx outtake;

    private IMU imu;

    //Outtake PIDF Values (To tune later)
    private double outtakeTargetVelocity = 0;


    @Override
    public void runOpMode() {

        // Hardware Maps

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        intake = hardwareMap.get(DcMotor.class, "intake");
        stage2 = hardwareMap.get(DcMotor.class, "stage2");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");

        imu = hardwareMap.get(IMU.class, "imu");



        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);



        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );

        imu.initialize(parameters);



        // Auto Button REMOVED

        //Telemetry
        telemetry.addLine("Blue TeleOp Initialized");
        telemetry.addLine("A = Automatic Position");
        telemetry.addLine("B = Intake");
        telemetry.addLine("RT = Outtake");
        telemetry.update();


        waitForStart();

        imu.resetYaw();



        while (opModeIsActive()) {


            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;


            double heading = imu
                    .getRobotYawPitchRollAngles()
                    .getYaw(AngleUnit.RADIANS);


            double rotX =
                    x * Math.cos(-heading)
                            - y * Math.sin(-heading);

            double rotY =
                    x * Math.sin(-heading)
                            + y * Math.cos(-heading);


            rotX *= 1.1;


            // Mecanum calculations
            double frontLeftPower = rotY + rotX + rx;
            double backLeftPower = rotY - rotX + rx;
            double frontRightPower = rotY - rotX - rx;
            double backRightPower = rotY + rotX - rx;


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



            // Telemetry

            telemetry.addData(
                    "Heading",
                    Math.toDegrees(heading)
            );

            telemetry.addData(
                    "Outtake Velocity",
                    outtake.getVelocity()
            );

            telemetry.addData(
                    "Outtake Target",
                    outtakeTargetVelocity
            );

            telemetry.update();
        }
    }
}