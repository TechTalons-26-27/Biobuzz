package org.firstinspires.ftc.teamcode.util;

import com.pedropathing.math.Pose;

public enum Cell {
    RED_AUDIENCE(Alliance.RED, 57.75, 57.75),
    RED_OTHER(Alliance.RED, 57.75, 83.75),
    BLUE_AUDIENCE(Alliance.BLUE, 83.75, 57.75),
    BLUE_OTHER(Alliance.BLUE, 83.75, 83.75);

    public final Alliance alliance;
    public final Pose pose;

    Cell(Alliance alliance, double x, double y) {
        this.alliance = alliance;
        this.pose = new Pose(x, y);
    }

    // Returns the closest cell belonging to the given alliance (other alliance's cells are ignored).
    public static Cell closest(Alliance alliance, Pose robotPose) {
        Cell best = null;
        double bestDist = Double.MAX_VALUE;
        for (Cell cell : values()) {
            if (cell.alliance != alliance) continue;
            double dist = Math.hypot(cell.pose.x() - robotPose.x(), cell.pose.y() - robotPose.y());
            if (dist < bestDist) {
                bestDist = dist;
                best = cell;
            }
        }
        return best;
    }
}
