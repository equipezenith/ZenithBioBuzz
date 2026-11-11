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
    public static MecanumConfig drivetrainConfig = new MecanumConfig(
            c -> {
                c.frontLeftName.set("motorde");   // nomes da SUA configuração
                c.backLeftName.set("motorte");
                c.frontRightName.set("motordd");
                c.backRightName.set("motortd");

                c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
                c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
                c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
                c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);

                c.manualBrakeMode.set(true);
            }
    );

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pod_odometria");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-7.297032273660496);
        c.yPodOffset.set(2.3183574826698603);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.4134190420528137);
                Controller secondaryTranslationalForward = Controller.proportional(0.1527472892782599);
                Controller primaryTranslationalLateral = Controller.proportional(0.5077346699879182);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1875943936402881);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.016352132311199377));
                c.brake.set(Controller.proportionalFeedforward(0.01389931246451947));

                c.headingFeedback.set(Controller.proportional(5.627404594030967));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.06821096049497345, 0.0028577455981502117));

                c.linearBrakeCoefficients.set(Matrix.diag(0.12197547830332664, 0.08670402429432199));
                c.quadraticBrakeCoefficients.set(Matrix.diag(2.791908625366665E-4, 0.0014007559460784776));

                c.maxAchievableForwardVelocity.set(63.77379500938433);
                c.maxAchievableStrafeVelocity.set(50.691293192847006);
                c.naturalForwardDeceleration.set(44.43434054362182);
                c.naturalStrafeDeceleration.set(82.18749342249922);
            }
    );

    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}