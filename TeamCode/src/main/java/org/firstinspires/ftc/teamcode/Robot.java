package org.firstinspires.ftc.teamcode;


import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.FusionLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;


public class Robot {
    Drive drive;
//    Intake intake;
//    Outtake outtake;
//    Limelight limelight;
    FusionLocalizer localizer;

    public void initSubsystems(HardwareMap hardwareMap, Pose startPose) {
        localizer = new FusionLocalizer(hardwareMap, startPose);
        drive = new Drive(hardwareMap, startPose);
//        limelight = new Limelight(hardwareMap, 0);

        //intake = new Intake(hardwareMap);
        //outtake = new Outtake(hardwareMap);
    }


    public void manualDrive(double forward, double lateral, double rotate) {
        drive.run(forward, lateral, rotate);
        localizer.run();
//        drive.updatePose(localizer.getPose());
    }


    public void start(double curTime) {
        localizer.start();
        drive.start(curTime);
    }


//    public void outtake(double outtakeTrigger) {
//        outtake.run(outtakeTrigger);
//    }
//
//
//    public void intake(boolean intakeButton) {
//        intake.run(intakeButton);
//    }


    public void telemetry(Telemetry telemetry) {
        drive.telemetry(telemetry);
    }

//    public void switchLLPipeline(int pipeline) {
//        limelight.switchPipeline(pipeline);
//    }

    public Pose getPose() {
        return drive.getPose();
    }

    public double track(double curTime, Pose targetPose) {
//        localizer.run();
        return drive.track(drive.getPose(), targetPose, curTime);
    }

    public void localize() {
        localizer.run();
        drive.updatePose(localizer.getPose());
    }
}