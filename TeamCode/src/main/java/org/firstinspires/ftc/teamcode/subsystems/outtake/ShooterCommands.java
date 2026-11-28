package org.firstinspires.ftc.teamcode.subsystems.outtake;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleSupplier;

/**
 * Ivy commands for autos. Distance is a supplier so it is recomputed from the live pose
 * every loop, e.g. robot::distanceToGoal.
 */
public final class ShooterCommands {

    private ShooterCommands() {}

    /** Spins the flywheel up to the shot RPM and finishes immediately; pair with a path to pre-spin. */
    public static Command spinUp(Shooter shooter, DoubleSupplier distance) {
        return Command.build()
                .setStart(() -> shooter.spinUp(distance.getAsDouble()))
                .setDone(() -> true);
    }

    /**
     * Shoots for durationMs: feeds only while the flywheel is at speed, then stops feeding
     * and idles the flywheel. The duration is also the timeout, so a jam can't stall auto.
     */
    public static Command shoot(Shooter shooter, DoubleSupplier distance, double durationMs) {
        ElapsedTime timer = new ElapsedTime();
        return Command.build()
                .setStart(timer::reset)
                .setExecute(() -> shooter.update(true, distance.getAsDouble()))
                .setDone(() -> timer.milliseconds() >= durationMs)
                .setEnd(end -> shooter.stopShooting())
                .requiring(shooter);
    }
}
