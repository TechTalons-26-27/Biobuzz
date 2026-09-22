package org.firstinspires.ftc.teamcode.config.subsystems.drive;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.pedro.Constants;

public class MecanumDrive {
    private final Follower follower;

    private boolean fieldOriented = false;

    public MecanumDrive(HardwareMap hardwareMap) {
        follower = Constants.create(hardwareMap);
    }

    public void loop(double forward, double lateral, double rotate, boolean changeModeButton) {
        toggleFieldOriented(changeModeButton);

        if (!fieldOriented) {
            ManualDrive.driveOrHold(
                    follower,
                    forward,
                    lateral,
                    rotate
            );
        }
        else {
            DrivePowers powers = ManualDrive.fieldCentric(
                    forward,
                    lateral,
                    rotate,
                    follower.pose().heading()
            );
            ManualDrive.driveOrHold(follower, powers);
        }

        follower.update();

    }

    public void toggleFieldOriented(boolean button) {
        if (button) {
            fieldOriented = !fieldOriented;
        }
    }
}
