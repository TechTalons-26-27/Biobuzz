package org.firstinspires.ftc.teamcode.config.subsystems.outtake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outtake {
    double targetVelocity = 1500;

    private DcMotorEx stage;
    private DcMotorEx outtake;
    double P = 0;
    double F = 0;

    public Outtake(HardwareMap hardwareMap) {
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");
        stage = hardwareMap.get(DcMotorEx.class, "stage");

        stage.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        outtake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // for now

        //for later
        //outtake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        //outtake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    public void loop(double outtakeTrigger) {
        if (outtakeTrigger > 0.2) {
            //outtake.setVelocity(targetVelocity);
            stage.setPower(1);
            outtake.setPower(outtakeTrigger);
        } else {
            stage.setPower(0);
            outtake.setPower(0);
        }

    }
}
