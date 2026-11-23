package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.subsystems.drive.BaseDrive;
import org.firstinspires.ftc.teamcode.subsystems.vision.Limelight;

@TeleOp(name="auto align")
public class AutoAlign extends OpMode {

    private Limelight limelight;
    private BaseDrive drive;

    // --------------------------- PD Controller ---------------------------
    double rkP = -0.0200;
    double dkP = 0;
    double rerror = 0;
    //double dTa = 0;
    double rlastError = 0;
    double dlastTa = 0;
    double rangleTolerance = 0.2;
    double targetTa = 70;
    double rkD = -0.0005;
    double dkD = 0;
    double rcurTime = 0;
    //double dcurTime = 0;
    double rlastTime = 0;
    //double dlastTime = 0;
    boolean tuningForward = false;   // false = tuning rotate gains, true = tuning forward gains

    // --------------------------- Drive ---------------------------

    double forward, strafe, rotate;

    // --------------------------- controller based PD tuning ---------------------------

    double[] stepSizes = {0.1, 0.01,0.001, 0.0001};
    int stepIndex = 2;

    @Override
    public void init() {
        limelight = new Limelight(hardwareMap,3);
        drive = new BaseDrive(hardwareMap);


        telemetry.addLine("Initialized");
    }

    public void start() {
        resetRuntime();
        rcurTime = getRuntime();
        //dcurTime = getRuntime();
    }

    @Override
    public void loop() {
        // --------------------------- get mecanum drive inputs ---------------------------

        forward = -gamepad1.left_stick_y;
        strafe = gamepad1.left_stick_x;
        rotate = gamepad1.right_stick_x;

        // --------------------------- get limelight input ---------------------------

        double Tx = limelight.getTx();
        double Ta = limelight.getTa();

        // --------------------------- auto align rotation logic ---------------------------

        if (gamepad1.left_trigger > 0.3) {
            if (Tx != -999) {

                rerror = Tx;

                if (Math.abs(rerror) < rangleTolerance) {
                    rotate = 0;
                } else {
                    double pTerm = rerror * rkP;

                    rcurTime = getRuntime();
                    double dT = rcurTime - rlastTime;

                    double dTerm = 0;

                    if (dT != 0) {
                        dTerm = ((rerror - rlastError) / dT) * rkD;
                    }

                    rotate = Range.clip(pTerm + dTerm, -0.4, 0.4);

                    rlastError = rerror;
                    rlastTime = rcurTime;
                }
            } else {
                rlastTime = getRuntime();
                rlastError = 0;

            }
        } else {
            rlastTime = getRuntime();
            rlastError = 0;
        }

        if (gamepad1.right_trigger > 0.3) {
            if (Ta != -999) {

                double derror = targetTa - Ta;   // positive = ball still far away

                if (derror <= 0) {
                    forward = 0;                 // reached (or passed) target size, stop
                } else {
                    // error shrinks as Ta grows, so output shrinks as the ball gets closer
                    forward = Range.clip(derror * dkP, -0.7, 0);
                }
            }
        }

        // drive motors
        drive.run(forward,strafe,rotate);

// update P and D on the fly
// 'B' button cycles through different step sizes for tuning precision
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length; // Modulo wraps index to 0
        }

// 'X' button switches between tuning rotate gains and forward gains
        if (gamepad1.xWasPressed()) {
            tuningForward = !tuningForward;
        }

        if (tuningForward) {
            if (gamepad1.dpadLeftWasPressed())  dkP -= stepSizes[stepIndex];
            if (gamepad1.dpadRightWasPressed()) dkP += stepSizes[stepIndex];
            if (gamepad1.dpadUpWasPressed())    dkD += stepSizes[stepIndex];
            if (gamepad1.dpadDownWasPressed())  dkD -= stepSizes[stepIndex];
        } else {
            if (gamepad1.dpadLeftWasPressed())  rkP -= stepSizes[stepIndex];
            if (gamepad1.dpadRightWasPressed()) rkP += stepSizes[stepIndex];
            if (gamepad1.dpadUpWasPressed())    rkD += stepSizes[stepIndex];
            if (gamepad1.dpadDownWasPressed())  rkD -= stepSizes[stepIndex];
        }

        // --------------------------- telemetry ---------------------------
        if (Tx != -999) {
            if (gamepad1.left_trigger > 0.3) {
                telemetry.addLine("AUTO ALIGN");
            }
            telemetry.addData("Rotate Error", rerror);
        } else {
            telemetry.addLine("MANUAL Rotate Mode");
        }
        telemetry.addData("Tx", Tx);
        telemetry.addData("Ta", Ta);
        telemetry.addData("Target Ta", targetTa);
        telemetry.addLine("-----------------------------");
        telemetry.addData("Now tuning", tuningForward ? "FORWARD (X to switch)" : "ROTATE (X to switch)");
        telemetry.addData((tuningForward ? "> " : "  ") + "Forward P", "%.4f (D-Pad L/R)", dkP);
        telemetry.addData((tuningForward ? "> " : "  ") + "Forward D", "%.4f (D-Pad U/D)", dkD);
        telemetry.addData((!tuningForward ? "> " : "  ") + "Rotate P", "%.4f (D-Pad L/R)", rkP);
        telemetry.addData((!tuningForward ? "> " : "  ") + "Rotate D", "%.4f (D-Pad U/D)", rkD);
        telemetry.addData("Step Size", "%.4f (B Button)", stepSizes[stepIndex]);
        telemetry.update();
    }
}