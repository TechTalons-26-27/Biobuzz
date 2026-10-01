package org.firstinspires.ftc.teamcode.opmode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.subsystems.drive.BaseDrive;
import org.firstinspires.ftc.teamcode.config.subsystems.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.config.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.config.subsystems.outtake.Outtake;

@TeleOp(name="all these tards")
public class initialTeleOp extends OpMode {

    //TODO: USE BASEDRIVE IF NOT TUNED
    //BaseDrive drive;
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
        forward = gamepad1.left_stick_x;
        lateral = -gamepad1.left_stick_y;
        rotate = -gamepad1.right_stick_x;

        drive.loop(forward, lateral, rotate);
        //drive.loop(forward, lateral, rotate, gamepad1.bWasPressed());

        if (gamepad1.rightTriggerWasPressed()) {
            outtake.loop(gamepad1.right_trigger);
            intake.loop(true);
        } else {
            intake.loop(gamepad1.aWasPressed());
        }
    }
}