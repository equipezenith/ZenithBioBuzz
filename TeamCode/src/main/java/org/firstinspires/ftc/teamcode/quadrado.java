package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "quadrado")

public class quadrado extends OpMode {
    funcoes robo;
    @Override
    public void init() {
        robo = new funcoes(hardwareMap, telemetry);
        if (robo.vision_cor.getVisionPortal() != null) {
            FtcDashboard.getInstance().startCameraStream(robo.vision_cor.getVisionPortal(), 30);
        }
    }

    @Override
    public void loop() {
        robo.atualizarServoPorCor(robo.novoCiclo);
        if (gamepad1.right_bumper) {
            robo.novoCiclo = true;
        }
    }
}
