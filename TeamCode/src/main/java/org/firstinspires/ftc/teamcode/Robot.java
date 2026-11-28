package org.firstinspires.ftc.teamcode;


import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.localization.Localizer;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subsystems.drive.Drive;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.subsystems.outtake.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.outtake.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.turret.Turret;
import org.firstinspires.ftc.teamcode.util.Alliance;
import org.firstinspires.ftc.teamcode.util.Cell;
import org.firstinspires.ftc.teamcode.util.PoseStorage;


public class Robot {
    Drive drive;
    Intake intake;
    Outtake outtake;
    Shooter shooter;
//    Limelight limelight;
    Localizer localizer;
    Turret turret;
    Follower follower;
    Alliance alliance;


    public void initSubsystems(HardwareMap hardwareMap, Pose startPose, Alliance alliance) {
        this.alliance = alliance;
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        localizer = new Localizer(hardwareMap, follower, startPose);
        drive = new Drive(follower, alliance.offsetHeading);
        turret = new Turret(hardwareMap);
//        limelight = new Limelight(hardwareMap, 0);

        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        shooter = new Shooter(outtake, intake);
    }


    public void drive(double forward, double lateral, double rotate) {
        drive.update(forward, lateral, rotate);
    }


    public void start(double curTime) {
        localizer.start();
        drive.start(curTime);
    }


    public void score(boolean shootHeld, boolean intakeHeld) {
        boolean shooting = shooter.update(shootHeld, distanceToGoal());
        if (!shooting) {
            intake.update(intakeHeld);
        }
    }


    public double distanceToGoal() {
        Pose pose = drive.getPose();
        Pose goal = Cell.closest(alliance, pose).pose;
        return Math.hypot(goal.x() - pose.x(), goal.y() - pose.y());
    }


    public Follower getFollower() {
        return follower;
    }


    public void savePose() {
        PoseStorage.currentPose = drive.getPose();
    }


    public void telemetry(Telemetry telemetry) {
        drive.telemetry(telemetry);
        shooter.telemetry(telemetry);
        intake.telemetry(telemetry);
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

    public void turret(Alliance alliance) {
        Pose pose = drive.getPose();
        Cell target = Cell.closest(alliance, pose);
        turret.update(pose, target.pose);
    }

    public void localize() {
        localizer.run();
    }
}