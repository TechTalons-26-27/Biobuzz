package org.firstinspires.ftc.teamcode.util;

public enum Alliance {
    RED(0),
    BLUE(Math.PI);

    /** Field heading (rad) the robot drives toward when the driver pushes the left stick forward. */
    public final double offsetHeading;

    Alliance(double offsetHeading) {
        this.offsetHeading = offsetHeading;
    }
}
