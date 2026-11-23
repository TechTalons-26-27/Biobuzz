package org.firstinspires.ftc.teamcode.teleOp;


import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.PoseStorage;


/**
 * Main teleop. Not an opmode itself: BlueTeleOp and RedTeleOp pick the alliance.
 * The turret tracks only the closest cell of that alliance.
 */
public abstract class BaseTeleOp extends OpMode {

    protected final Alliance alliance;
    protected Robot robot;
    protected double forward, lateral, rotate;

    protected BaseTeleOp(Alliance alliance) {
        this.alliance = alliance;
    }

    @Override
    public void init() {
        // Starts where auto left off (see PoseStorage).
        Pose startPose = PoseStorage.currentPose;

        robot = new Robot();
        robot.initSubsystems(hardwareMap, startPose);
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

        robot.manualDrive(forward, lateral, rotate);

        robot.turret(alliance);

        robot.telemetry(telemetry);
    }
}
