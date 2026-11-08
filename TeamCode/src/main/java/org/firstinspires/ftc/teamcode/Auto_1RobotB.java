package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name="Auto1Blue")
@Disabled
public class Auto_1RobotB extends LinearOpMode {

    funcoes robo;
    @Override
    public void runOpMode() {
        robo = new funcoes(hardwareMap, telemetry);
        

    }
}
