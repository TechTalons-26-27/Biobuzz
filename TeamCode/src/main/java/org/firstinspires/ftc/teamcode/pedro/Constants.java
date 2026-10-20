package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static Follower create(HardwareMap h) {
        // return new Follower(Drivetrain, Localizer, Foresight);
        return null;
    }

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotor.Direction.FORWARD);
        c.frontRightDirection.set(DcMotor.Direction.REVERSE);
        c.backLeftDirection.set(DcMotor.Direction.FORWARD);
        c.backRightDirection.set(DcMotor.Direction.REVERSE);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(0.7609923054852824);
        c.yPodOffset.set(5.1774452990434305);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.34171607813159594);
                Controller secondaryTranslationalForward = Controller.proportional(0.12625496004785225);
                Controller primaryTranslationalLateral = Controller.proportional(0.5479954417602616);
                Controller secondaryTranslationalLateral = Controller.proportional(0.20246967302253424);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.012871965832389493));
                c.brake.set(Controller.proportionalFeedforward(0.010941170957531068));

                c.headingFeedback.set(Controller.proportional(6.805705018714027));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.07297716767375227, 0.00571764489052405));

                c.linearBrakeCoefficients.set(Matrix.diag(0.11972185430605693, 0.062230288175772926));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0012847104740659886, 0.0019836256291352714));

                c.maxAchievableForwardVelocity.set(79.12655957022207);
                c.maxAchievableStrafeVelocity.set(57.57032549066906);
                c.naturalForwardDeceleration.set(36.21525996195826);
                c.naturalStrafeDeceleration.set(65.11727896495813);
            }
    );
}