package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Alliance;

@TeleOp(name = "Red TeleOp", group = "TeleOp")
public class RedTeleOp extends BaseTeleOp {
    public RedTeleOp() {
        super(Alliance.RED);
    }
}
