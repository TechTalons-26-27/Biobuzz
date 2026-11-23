package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.drive.BaseDrive;

/**
 * Robot-centric mecanum drive tester that only depends on BaseDrive.
 * Use it to check motor directions and wiring.
 */
@TeleOp(name = "Mecanum Tester", group = "Test")
public class MecanumTester extends OpMode {

    private BaseDrive drive;

    @Override
    public void init() {
        drive = new BaseDrive(hardwareMap);

        telemetry.addLine("Mecanum Tester ready");
        telemetry.addLine("Left stick = drive/strafe, right stick X = rotate");
        telemetry.update();
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        drive.run(forward, lateral, rotate);

        telemetry.addData("Forward", "%.2f", forward);
        telemetry.addData("Strafe", "%.2f", lateral);
        telemetry.addData("Rotate", "%.2f", rotate);
        telemetry.addLine();
        drive.telemetry(telemetry);
        telemetry.update();
    }

    @Override
    public void stop() {
        drive.run(0, 0, 0);
    }
}
