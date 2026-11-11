package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Turret Tester")
public class turretTest extends OpMode {

    private Servo turretServo;

    private DcMotor turretMotor;

    private double turretPower;

    public void init() {
        turretServo = hardwareMap.get(Servo.class, "turretservo");
        turretMotor = hardwareMap.get(DcMotor.class, "turretmotor");

    }

    public void loop() {

        turretPower = 0.5;

        if (gamepad2.a) {
            turretPower += 1;
        }

        if (gamepad2.b) {
            turretPower -= 1;
        }

        if (gamepad2.right_trigger > 0.05) {
            turretMotor.setPower(turretPower);
        }

        if (gamepad2.left_bumper) {
            turretServo.setPosition(1);
        }

        if (gamepad2.right_bumper) {
            turretServo.setPosition(0);
        }
    }
}
