package org.firstinspires.ftc.teamcode.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {
    private DcMotor intake;
    private DcMotor stage;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class,"intake");
        stage = hardwareMap.get(DcMotor.class, "stage");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void update(boolean intakeButton) {
        if (intakeButton) {
            intake.setPower(0.5);
            stage.setPower(0.5);
        } else {
            intake.setPower(0);
            stage.setPower(0);
        }
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Intake & Stage Power", intake.getPower());
    }
}