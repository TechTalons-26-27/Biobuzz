package org.firstinspires.ftc.teamcode.config.subsystems.drive;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.pedro.Constants;

public class PedroDrive {
    private final Follower follower;

    private boolean fieldOriented = false;

    public PedroDrive(HardwareMap hardwareMap) {
        follower = Constants.create(hardwareMap);
    }

    public void run(double forward, double lateral, double rotate, boolean changeModeButton) {
        toggleFieldOriented(changeModeButton);

        if (!fieldOriented) { // normal drive
            ManualDrive.driveOrHold(
                    follower,
                    forward,
                    lateral,
                    rotate
            );
        }
        else { // field oriented
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
