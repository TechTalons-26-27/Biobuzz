package org.firstinspires.ftc.teamcode.opmode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.subsystems.drive.BaseDrive;
import org.firstinspires.ftc.teamcode.config.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.outtake.Outtake;

@TeleOp(name="Drive + Intake + Outtake")
public class initialTeleOp extends OpMode {

    //PedroDrive drive;
    BaseDrive drive;
    Intake intake;
    Outtake outtake;
    double forward, lateral, rotate;

    @Override
    public void init() {
        //drive = new BaseDrive(hardwareMap);
        drive = new BaseDrive(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
    }

    @Override
    public void loop() {
        forward = -gamepad1.left_stick_y;
        lateral = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        drive.run(forward, lateral, rotate);
        //drive.run(forward, lateral, rotate, gamepad1.bWasPressed());

        if (gamepad1.right_trigger > 0.2) {
            outtake.run(gamepad1.right_trigger);
            intake.run(true);
        } else {
            if (gamepad1.left_trigger > 0.2) {
                outtake.run(gamepad1.left_trigger);
            } else {
                outtake.run(0);
            }
            intake.run(gamepad1.a);
        }

        //drive.telemetry(telemetry);
        //intake.telemetry(telemetry);
        //outtake.telemetry(telemetry);
    }
}