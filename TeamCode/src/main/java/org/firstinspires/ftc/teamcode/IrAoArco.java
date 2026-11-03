package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Config
@TeleOp(name = "Ir ao Arco (botão A)", group = "Teste")
public class IrAoArco extends LinearOpMode {

    public static double START_X = 9;
    public static double START_Y = 9;
    public static double START_HEADING_DEG = 90;

    private Follower follower;
    private boolean aPrev = false;


    @Override
    public void runOpMode() {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG)));
        follower.update();

        telemetry.addLine("Pronto. Robô no canto inferior esquerdo. A = ir ao arco.");
        telemetry.update();

        waitForStart();

        Pose arcTarget = null;

        while (opModeIsActive()) {
            follower.update();
            Pose pose = follower.pose();

            boolean aNow = gamepad1.a;
            if (aNow && !aPrev && !follower.isBusy()) {
                arcTarget = ArcoHive.nearestArcPose(pose);
                if (ArcoHive.distance(pose, arcTarget) > ArcoHive.ON_ARC_TOLERANCE) {
                    follower.follow(ArcoHive.buildPath(pose, arcTarget));
                }
            }
            aPrev = aNow;

            if (!follower.isBusy()) {
                follower.manual(
                        -gamepad1.left_stick_y,
                        gamepad1.left_stick_x,
                        gamepad1.right_stick_x);
            }

            telemetry.addData("Estado", follower.isBusy() ? "Indo ao arco" : "Parado");
            telemetry.addData("Pose", "x %.1f  y %.1f  h %.1f°",
                    pose.x(), pose.y(), Math.toDegrees(pose.heading()));
            if (arcTarget != null) {
                telemetry.addData("Último alvo no arco", "x %.1f  y %.1f", arcTarget.x(), arcTarget.y());
            }
            telemetry.update();
        }
    }
}