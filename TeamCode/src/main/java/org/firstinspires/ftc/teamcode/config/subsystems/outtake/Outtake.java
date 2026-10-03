package org.firstinspires.ftc.teamcode.config.subsystems.outtake;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Outtake {
    double targetVelocity = 1500;

    private DcMotorEx outtake;
    double P = 0;
    double F = 0;

    public Outtake(HardwareMap hardwareMap) {
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");

        outtake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER); // for now
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setDirection(DcMotor.Direction.REVERSE);

        //for later
        //outtake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P,0,0,F);
        //outtake.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    public void run(double outtakeTrigger) {
        if (outtakeTrigger > 0.2) {
            //outtake.setVelocity(targetVelocity);
            outtake.setPower(1);
        } else {
            outtake.setPower(0.0);
        }

    }

    public void telemetry(Telemetry telemetry) {
        telemetry.addData("Outtake Power", outtake.getPower());
    }
}