package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "OpPrototipos")
public class Shooter_test extends OpMode {

    funcoes robo;
    @Override
    public void init() {
        robo = new funcoes(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        robo.shooter(telemetry);
    }
}
