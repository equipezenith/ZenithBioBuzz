package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Teste Visao + Dashboard")
public class TesteDashboardOpMode extends LinearOpMode {

    // 1. Apenas declara a variável aqui (não passe hardwareMap nem telemetry ainda!)
    private funcoes robot;

    @Override
    public void runOpMode() {

        // 2. Instancia o objeto 'robot' AQUI DENTRO, onde hardwareMap e telemetry já existem!
        robot = new funcoes(hardwareMap, telemetry);

        // 3. Redireciona a telemetria para a Driver Station E para o FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // 4. Transmite o vídeo da webcam ao vivo no Dashboard (30 FPS)
        if (robot.vision.getVisionPortal() != null) {
            FtcDashboard.getInstance().startCameraStream(robot.vision.getVisionPortal(), 30);
        }

        waitForStart();

        while (opModeIsActive()) {

            // Atualiza o estado do servo baseado na cor
            robot.atualizarServoPorCor();

            // Telemetria detalhada para diagnóstico
            telemetry.addData("--- STATUS ---", "");
            telemetry.addData("Cor Detectada", robot.vision.getDetectedColor());
            telemetry.addData("Pixels Vermelhos", robot.vision.getRedPixels());
            telemetry.addData("Pixels Azuis", robot.vision.getBluePixels());
            telemetry.addData("Posicao do Servo", robot.indexer.getPosition());
            telemetry.update();
        }

        robot.stop();
    }
}