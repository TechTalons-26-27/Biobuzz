package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.outtake.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.vision.Limelight;

public class Robot {
    Drive drive;
    Intake intake;
    Outtake outtake;
    Limelight limelight;

    public void initSubsystems(HardwareMap hardwareMap, Telemetry telemetry, Pose startPose) {
        drive = new Drive(hardwareMap, startPose);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        limelight = new Limelight(hardwareMap, telemetry, 0);

    }

    public void drive(double forward, double lateral, double rotate) {
        drive.run(forward, lateral, rotate);
    }

    public void outtake(double outtakeTrigger) {
        outtake.run(outtakeTrigger);
    }

    public void intake(boolean intakeButton) {
        intake.run(intakeButton);
    }
}
