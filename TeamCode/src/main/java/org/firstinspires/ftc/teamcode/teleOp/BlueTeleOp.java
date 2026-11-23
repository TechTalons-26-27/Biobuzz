package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.Alliance;

@TeleOp(name = "Blue TeleOp", group = "TeleOp")
public class BlueTeleOp extends BaseTeleOp {
    public BlueTeleOp() {
        super(Alliance.BLUE);
    }
}
