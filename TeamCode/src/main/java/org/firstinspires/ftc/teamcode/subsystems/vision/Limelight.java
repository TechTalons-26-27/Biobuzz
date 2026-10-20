
package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Limelight {

    private final Limelight3A limelight3A;
    private final Telemetry telemetry;

    private long lastTimestamp = Long.MIN_VALUE;

    public Limelight(
            HardwareMap hardwareMap,
            Telemetry telemetry,
            int pipeline) {

        this.telemetry = telemetry;

        limelight3A =
                hardwareMap.get(Limelight3A.class, "limelight");

        limelight3A.setPollRateHz(100);
        limelight3A.pipelineSwitch(pipeline);
        limelight3A.start();
    }

    public Pose getNewFieldPose() {

        LLResult result = limelight3A.getLatestResult();

        if (result == null || !result.isValid()) {
            return null;
        }

        if (result.getBotposeTagCount() < 1) {
            return null;
        }

        // Reject stale results.
        if (result.getStaleness() > 250) {
            return null;
        }

        long timestamp = result.getControlHubTimeStampNanos();

        if (timestamp <= 0 || timestamp <= lastTimestamp) {
            return null;
        }

        lastTimestamp = timestamp;

        Pose3D botPose = result.getBotpose();

        if (botPose == null) {
            return null;
        }

        double xMeters =
                botPose.getPosition().x;

        double yMeters =
                botPose.getPosition().y;

        // Convert meters to inches.
        double xInches = DistanceUnit.INCH.fromMeters(xMeters);
        double yInches = DistanceUnit.INCH.fromMeters(yMeters);

        if (!Double.isFinite(xInches) ||
                !Double.isFinite(yInches)) {
            return null;
        }

        // IMPORTANT:
        // Limelight and PedroPathing must use the SAME
        // field origin, axis directions and units.
        // Apply the calibrated coordinate transform here
        // before enabling vision fusion.

        return new Pose(xInches, yInches, 0);
    }

    public double getTx() {
        LLResult result = limelight3A.getLatestResult();
        return result != null && result.isValid()
                ? result.getTx() : -999;
    }

    public double getTy() {
        LLResult result = limelight3A.getLatestResult();
        return result != null && result.isValid()
                ? result.getTy() : -999;
    }

    public double getTa() {
        LLResult result = limelight3A.getLatestResult();
        return result != null && result.isValid()
                ? result.getTa() : -999;
    }

    public void stop() {
        limelight3A.stop();
    }
}
