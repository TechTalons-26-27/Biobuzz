package org.firstinspires.ftc.teamcode.config.subsystems.intake;

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

    public void run(boolean intakeButton) {
        if (intakeButton) {
            intake.setPower(1);
            stage.setPower(0.2);
        } else {
            intake.setPower(0);
            stage.setPower(0);
        }
    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Intake Power:", intake.getPower());
        telemetry.addData("Stage Power:", stage.getPower());
    }
}