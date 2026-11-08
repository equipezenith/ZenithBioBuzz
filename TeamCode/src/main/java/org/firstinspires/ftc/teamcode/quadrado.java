package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "quadrado")

public class quadrado extends OpMode {
    funcoes robo;
    @Override
    public void init() {
        robo = new funcoes(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        robo.atualizarServoPorCor(robo.novoCiclo);
        if (gamepad1.right_bumper) {
            robo.novoCiclo = true;
        }
    }
}
