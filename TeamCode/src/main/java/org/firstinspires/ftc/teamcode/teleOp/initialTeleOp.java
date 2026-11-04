package org.firstinspires.ftc.teamcode.teleOp;


import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.PoseStorage;


@TeleOp(name="kalman test")
public class initialTeleOp extends OpMode {

    Robot robot;
    double forward, lateral, rotate;

    double kP = 0;
    double kD = 0;

    double[] stepSizes = {0.1, 0.01,0.001, 0.0001};
    int stepIndex = 2;

    final PoseFactory p = PoseFactory.radians();
    Pose startPose = p.of(0,0,0);
    Pose targetPose = p.of(0,0,0);

    @Override
    public void init() {
        robot = new Robot();
        robot.initSubsystems(hardwareMap, targetPose);
    }


    @Override
    public void start() {
        resetRuntime();
        robot.start(getRuntime());
    }


    @Override
    public void loop() {

        robot.localize();

        forward = -gamepad1.left_stick_y;
        lateral = -gamepad1.left_stick_x;
        rotate = -gamepad1.right_stick_x;

        if (gamepad1.y) {
            rotate = robot.track(getRuntime(), targetPose);
        }

        robot.manualDrive(forward, lateral, rotate);

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