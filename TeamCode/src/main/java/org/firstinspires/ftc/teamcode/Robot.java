package org.firstinspires.ftc.teamcode;


import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.FusionLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.outtake.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.vision.Limelight;


public class Robot {
    Drive drive;
    Intake intake;
    Outtake outtake;
    Limelight limelight;
    FusionLocalizer localizer;


    public void initSubsystems(HardwareMap hardwareMap, Pose startPose) {
        drive = new Drive(hardwareMap, startPose);
        limelight = new Limelight(hardwareMap, 0);
        localizer = new FusionLocalizer(hardwareMap, startPose);

        //intake = new Intake(hardwareMap);
        //outtake = new Outtake(hardwareMap);



    }


    public void drive(double forward, double lateral, double rotate) {
        drive.run(forward, lateral, rotate);
    }


    public void start() {

    }

    // Inside Robot.java
    public void loop() {
        localizer.run(); // Run prediction + vision correction step
    }


    /*public void outtake(double outtakeTrigger) {
        outtake.run(outtakeTrigger);
    }*/


    /*public void intake(boolean intakeButton) {
        intake.run(intakeButton);
    }*/


    public void telemetry(Telemetry telemetry) {
        drive.telemetry(telemetry);
        localizer.telemetry(telemetry);
        telemetry.update();
    }

    public void switchLLPipeline(int pipeline) {
        limelight.switchPipeline(pipeline);
    }

    public Pose getPose() {
        return localizer.getPose();
    }
}

