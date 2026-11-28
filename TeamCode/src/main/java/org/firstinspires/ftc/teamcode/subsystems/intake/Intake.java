package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.HardwareNames;

public class Intake {
    // Collecting balls (driver button).
    private static final double INTAKE_POWER = 0.5;
    private static final double INTAKE_STAGE_POWER = 0.5;

    // Pushing balls into the flywheel while shooting. TODO: TUNE
    private static final double FEED_INTAKE_POWER = 1.0;
    private static final double FEED_STAGE_POWER = 1.0;

    // Stage power while shooting but the flywheel isn't at speed yet.
    // 0 = just stop; make it slightly negative (e.g. -0.2) if balls creep into the flywheel early.
    private static final double HOLD_STAGE_POWER = 0.0;

    private final DcMotor intake;
    private final DcMotor stage;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class, HardwareNames.INTAKE);
        stage = hardwareMap.get(DcMotor.class, HardwareNames.STAGE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        stage.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update(boolean intakeButton) {
        if (intakeButton) {
            intake.setPower(INTAKE_POWER);
            stage.setPower(INTAKE_STAGE_POWER);
        } else {
            stop();
        }
    }

    public void feed(boolean ready) {
        if (ready) {
            intake.setPower(FEED_INTAKE_POWER);
            stage.setPower(FEED_STAGE_POWER);
        } else {
            intake.setPower(0);
            stage.setPower(HOLD_STAGE_POWER);
        }
    }

    /** Runs intake and stage backwards to clear a jam. */
    public void reverse() {
        intake.setPower(-INTAKE_POWER);
        stage.setPower(-INTAKE_STAGE_POWER);
    }

    public void stop() {
        intake.setPower(0);
        stage.setPower(0);
    }


    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Intake power", intake.getPower());
        telemetry.addData("Stage power", stage.getPower());
    }
}
