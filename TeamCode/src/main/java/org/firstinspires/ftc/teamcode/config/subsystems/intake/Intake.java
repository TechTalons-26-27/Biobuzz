package org.firstinspires.ftc.teamcode.config.subsystems.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor intake;

    public Intake(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotor.class,"intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void loop(boolean intakeButton) {
        if (intakeButton) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }
    }
}
