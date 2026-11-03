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
        c.xPodOffset.set(-5.4464799775851995);
        c.yPodOffset.set(-1.3605159849632444);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2777609044007932);
                Controller secondaryTranslationalForward = Controller.proportional(0.10262523226803628);
                Controller primaryTranslationalLateral = Controller.proportional(0.3740789220652431);
                Controller secondaryTranslationalLateral = Controller.proportional(0.13821216613021864);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01593345861286253));
                c.brake.set(Controller.proportionalFeedforward(0.013543439820933149));

                c.headingFeedback.set(Controller.proportional(4.869188084703567));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0691162337161977, 0.004079427372181645));

                c.linearBrakeCoefficients.set(Matrix.diag(0.11025371156082914, 0.09423115534968257));
                c.quadraticBrakeCoefficients.set(Matrix.diag(2.3311856546227015E-4, 7.282978101362243E-4));

                c.maxAchievableForwardVelocity.set(68.66650228257674);
                c.maxAchievableStrafeVelocity.set(55.823226342640645);
                c.naturalForwardDeceleration.set(50.504265634428364);
                c.naturalStrafeDeceleration.set(70.86830581337239);
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