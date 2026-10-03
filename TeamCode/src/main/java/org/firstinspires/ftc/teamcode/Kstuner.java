package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Kstuner extends OpMode {
    // Declarar o objeto sem o instanciar imediatamente
    private funcoes robo;

    public double kS = 0.072;
    double[] increments = {0.000001, 0.00001, 0.0001, 0.001, 0.01};
    int incrementIdx = 4;

    @Override
    public void init() {
        // Instanciar o robo dentro do init(), onde hardwareMap e telemetry já não são null
        robo = new funcoes(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        if (gamepad1.dpadRightWasPressed() && incrementIdx < increments.length - 1) {
            incrementIdx++;
        } else if (gamepad1.dpadLeftWasPressed() && incrementIdx > 0) {
            incrementIdx--;
        }

        double currentStep = increments[incrementIdx];

        // Altera o kS
        if (gamepad1.dpadUpWasPressed()) { kS += currentStep; }
        if (gamepad1.dpadDownWasPressed()) { kS -= currentStep; }

        robo.motor_sho.setPower(kS);

        telemetry.addData("Step Atual", "%.6f", currentStep);
        telemetry.addData("kS", "%.6f", kS);
        telemetry.addData("Velocidade Atual", robo.motor_sho.getVelocity());
        telemetry.update();
    }
}