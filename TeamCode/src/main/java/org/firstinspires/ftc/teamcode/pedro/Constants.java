package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static MecanumConfig driveConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("fl");
                c.backLeftName.set("rl");
                c.frontRightName.set("fr");
                c.backRightName.set("rr");

                c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
                c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
            }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(4.444556649275652);
        c.yPodOffset.set(1.6531720499354083);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.31925899376491623);
                Controller secondaryTranslationalForward = Controller.proportional(0.11795766743870993);
                Controller primaryTranslationalLateral = Controller.proportional(0.4647635579258282);
                Controller secondaryTranslationalLateral = Controller.proportional(0.17171771594260707);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.014088808939965258));
                c.brake.set(Controller.proportionalFeedforward(0.011975487598970469));

                c.headingFeedback.set(Controller.proportional(4.535912627240781));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04772139137820448, 0.007718295603061809));

                c.linearBrakeCoefficients.set(Matrix.diag(0.09119808150031468, 0.04531984424640593));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.001694355987206379, 0.002374481724399552));

                c.maxAchievableForwardVelocity.set(74.23940397550857);
                c.maxAchievableStrafeVelocity.set(55.37714011726223);
                c.naturalForwardDeceleration.set(35.34879357495383);
                c.naturalStrafeDeceleration.set(67.9316036946485);
            }
    );


    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, driveConfig),
                new Foresight(foresightConfig)
        );
    }
}