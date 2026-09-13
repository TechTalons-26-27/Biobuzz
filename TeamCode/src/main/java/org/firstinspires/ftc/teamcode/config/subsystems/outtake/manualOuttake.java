package org.firstinspires.ftc.teamcode.config.subsystems.outtake;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class manualOuttake {

    private DcMotorEx outtakeMotor, intakeMotor, stageMotor;
    private ElapsedTime timer = new ElapsedTime();
    private State state = State.IDLE;

    public void init(HardwareMap hardwareMap) {
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtake");
        stageMotor = hardwareMap.get(DcMotorEx.class, "stage");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");

        outtakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        outtakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        stageMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    // call once to start
    public void start() {
        timer.reset();
        state = State.SPIN_UP;
    }

    // call every loop
    public void update() {
        switch (state) {

            case IDLE:
                break;

            case SPIN_UP:
                outtakeMotor.setPower(1);
                intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
                stageMotor.setDirection(DcMotorEx.Direction.REVERSE);
                intakeMotor.setPower(1);
                stageMotor.setPower(1);

                if (timer.seconds() > 2) {
                    timer.reset();
                    state = State.FEED_FORWARD;
                }
                break;

            case FEED_FORWARD:
                intakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
                stageMotor.setDirection(DcMotorEx.Direction.FORWARD);

                if (timer.seconds() > 2) {
                    state = State.DONE;
                }
                break;

            case DONE:
                outtakeMotor.setPower(0);
                intakeMotor.setPower(0);
                stageMotor.setPower(0);
                state = State.IDLE;
                break;
        }
    }

    public void runOuttake(float power) {
        outtakeMotor.setPower(power);
        intakeMotor.setPower(power);
        stageMotor.setPower(power);
    }

    public void ramp() {
        outtakeMotor.setPower(1);
    }

    public boolean isBusy() {
        return state != State.IDLE;
    }

    private enum State {
        IDLE,
        SPIN_UP,
        FEED_REVERSE,
        FEED_FORWARD,
        DONE
    }
}