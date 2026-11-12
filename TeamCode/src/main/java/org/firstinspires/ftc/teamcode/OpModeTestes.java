package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "OPmodeTestes")
public class OpModeTestes extends OpMode {

    funcoes robo;
    @Override
    public void init() {
        robo = new funcoes(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        robo.atualizar();
        robo.andar(gamepad1);
        robo.alinharHive(gamepad1.a);
        robo.atualizarServoPorCor(robo.novoCiclo);
        if (gamepad2.left_trigger >= 0.5) {
            robo.shooterOPNectar(telemetry);
        } else if (gamepad2.right_trigger >= 0.5){
            robo.shooterOPPolem(telemetry);
        }else {
            robo.motor_shoN.setPower(0);
            robo.motor_shoP.setPower(0);
            robo.motor_tra.setPower(0);
        }
        robo.intake_Off(gamepad2 ,telemetry);
        robo.intake_Off(gamepad1, telemetry);
        robo.intake_Off_inv(gamepad2, telemetry);

    }
}
