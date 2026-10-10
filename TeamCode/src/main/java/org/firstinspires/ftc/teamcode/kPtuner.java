package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class kPtuner extends OpMode {
    // Declarar o objeto sem o instanciar imediatamente
    private funcoes robo;

    public double kV = 0.000166;

    public double kS = 0.072;
    public double kP = 0.00002;

    public double goalRPM = 2400;
    double[] increments = {0.000001, 0.00001, 0.0001, 0.001, 0.01};
    int incrementIdx = 2;

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

        // Altera o kP
        if (gamepad1.dpadUpWasPressed()) {
            kP += currentStep;
        }
        if (gamepad1.dpadDownWasPressed()) {
            kP -= currentStep;
        }

        double feedForward = (kV * goalRPM) + kS;
        double error = goalRPM - (robo.motor_sho.getVelocity() * 60) / 28;
        double feedBack = error * kP;
        robo.motor_sho.setPower(feedForward + feedBack);
        robo.motor_tra.setPower(0.4);

        telemetry.addData("Step Atual", "%.6f", currentStep);
        telemetry.addData("kP", "%.6f", kP);
        telemetry.addData("error", error);
        telemetry.addData("RPM Atual", (robo.motor_sho.getVelocity() * 60) / 28);
        telemetry.update();
    }
}
