package org.firstinspires.ftc.teamcode.teleOp;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.PoseStorage;


@TeleOp(name="kalman test")
public class initialTeleOp extends OpMode {


    Robot robot;
    double forward, lateral, rotate;


    @Override
    public void init() {
        robot = new Robot();
        robot.initSubsystems(hardwareMap, PoseStorage.currentPose);
    }


    @Override
    public void start() {
        robot.start();
    }


    @Override
    public void loop() {
        forward = -gamepad1.left_stick_y;
        lateral = -gamepad1.left_stick_x;
        rotate = -gamepad1.right_stick_x;


        robot.drive(forward, lateral, rotate);

        robot.loop();

//        if (gamepad1.right_trigger > 0.2) {
//            robot.outtake(gamepad1.right_trigger);
//            robot.intake(true);
//        } else {
//            if (gamepad1.left_trigger > 0.2) {
//                robot.outtake(gamepad1.left_trigger);
//            } else {
//                robot.outtake(0);
//            }
//            robot.intake(gamepad1.a);
//        }


        robot.telemetry(telemetry);
    }
}