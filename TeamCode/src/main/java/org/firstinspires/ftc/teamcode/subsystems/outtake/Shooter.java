package org.firstinspires.ftc.teamcode.subsystems.outtake;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.intake.Intake;

/**
 * Hold-to-shoot. While shooting, the flywheel targets the shot-table RPM for the distance
 * and the intake/stage feed balls only while the flywheel is at speed, so every ball
 * leaves at the right speed and the feeder pauses on its own while the flywheel recovers.
 * Same call works for teleop (trigger held) and auto (ShooterCommands).
 */
public class Shooter {

    private final Outtake outtake;
    private final Intake intake;
    private double lastDistance = 0;

    public Shooter(Outtake outtake, Intake intake) {
        this.outtake = outtake;
        this.intake = intake;
    }

    /**
     * Call every loop. Returns true while shooting; in that case the shooter owns the
     * intake and stage motors and nothing else should set them this loop.
     */
    public boolean update(boolean shoot, double distance) {
        lastDistance = distance;
        if (shoot) {
            outtake.aimAt(distance);
            intake.feed(outtake.isAtSpeed());
        } else {
            outtake.idle();
        }
        return shoot;
    }

    /** Pre-spin to the shot RPM (e.g. while driving to the shooting spot in auto) without feeding. */
    public void spinUp(double distance) {
        lastDistance = distance;
        outtake.aimAt(distance);
    }

    /** Stop feeding and drop the flywheel back to idle. */
    public void stopShooting() {
        intake.stop();
        outtake.idle();
    }

    public boolean isAtSpeed() {
        return outtake.isAtSpeed();
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Distance to goal (in)", "%.1f", lastDistance);
        outtake.telemetry(telemetry);
    }
}
