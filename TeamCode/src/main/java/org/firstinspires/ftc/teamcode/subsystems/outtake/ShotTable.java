package org.firstinspires.ftc.teamcode.subsystems.outtake;


public final class ShotTable {

    // TODO: TUNE. Placeholder values, replace with tested ones.
    private static final double[][] TABLE = {
            // distance (in), RPM
            {36, 2600},
            {48, 2850},
            {60, 3100},
            {72, 3400},
            {96, 3900},
    };

    private ShotTable() {}

    public static double rpmFor(double distance) {
        if (distance <= TABLE[0][0]) return TABLE[0][1];
        for (int i = 1; i < TABLE.length; i++) {
            if (distance <= TABLE[i][0]) {
                double d0 = TABLE[i - 1][0], r0 = TABLE[i - 1][1];
                double d1 = TABLE[i][0], r1 = TABLE[i][1];
                return r0 + (distance - d0) / (d1 - d0) * (r1 - r0);
            }
        }
        return TABLE[TABLE.length - 1][1];
    }
}
