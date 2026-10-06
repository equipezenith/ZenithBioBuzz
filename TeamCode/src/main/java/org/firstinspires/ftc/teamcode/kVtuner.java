package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class kVtuner extends OpMode {
    // Declarar o objeto sem o instanciar imediatamente
    private funcoes robo;

    public double kV = 0.000166;

    public double kS = 0.072;

    public double goalRPM = 2400;
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
        if (gamepad1.dpadUpWasPressed()) { kV += currentStep; }
        if (gamepad1.dpadDownWasPressed()) { kV -= currentStep; }

        double power = (kV * goalRPM) + kS;
        robo.motor_sho.setPower(power);

        telemetry.addData("Step Atual", "%.6f", currentStep);
        telemetry.addData("kV", "%.6f", kV);
        telemetry.addData("RPM Atual", (robo.motor_sho.getVelocity() * 60) / 28);
        telemetry.update();
    }
}
