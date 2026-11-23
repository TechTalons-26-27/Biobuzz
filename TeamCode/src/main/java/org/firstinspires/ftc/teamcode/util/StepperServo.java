package org.firstinspires.ftc.teamcode.util;

public class StepperServo {
    public static double toPosition(double rad, double range, double offset) {
        return Math.max(0, Math.min(1, (rad - offset) / range));
    }
}