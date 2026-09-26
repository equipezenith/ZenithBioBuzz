package org.firstinspires.ftc.teamcode;
import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class funcoes extends chassi  {
    int atag0;
    int atag1;
    int atag2;
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;
    public Sistema_visao vision = new Sistema_visao();

    DcMotorEx motor_in;

    public boolean in_state = false;

    DcMotorEx motor_sho;
    Servo indexer;

    public funcoes (HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap);
        motor_sho =  hardwareMap.get(DcMotorEx.class, "motorSho");
        motor_sho.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_in =  hardwareMap.get(DcMotorEx.class, "motorIn");
        indexer = hardwareMap.get(Servo.class, "indexer");
        indexer.setPosition(0.0);
        vision.init(hardwareMap);

        //Setup da camera para sensor de cor
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }


    public void intake_Off (Gamepad gamepad2, Telemetry telemetry) {
        if (gamepad2.left_trigger > 0) {
            motor_in.setDirection(DcMotorSimple.Direction.REVERSE);
            motor_in.setPower(1);
            in_state = true;
        } else {
            motor_in.setPower(0);
            in_state = false;
        }
    }

    public void intake_Off_inv (Gamepad gamepad2, Telemetry telemetry) {
        if (gamepad2.left_bumper) {
            motor_in.setPower(1);
            in_state = true;
        } else {
            motor_in.setPower(0);
            in_state = false;
        }
    }

    public void shooter(Telemetry telemetry){
        double ticks_per_second = (2400/60.0) * 28;
        boolean velocidade_MAX = false;
            if (velocidade_MAX && motor_sho.getVelocity() < ticks_per_second * 0.90){
                motor_sho.setPower(1);
            }
            else {
                motor_sho.setVelocity(1 * ticks_per_second);
                if (motor_sho.getVelocity() >= ticks_per_second * 0.97) {
                    velocidade_MAX = true;
                }
            }
            telemetry.addData("Velocidade Atual", motor_sho.getVelocity());
            telemetry.addData("Velocidade Alvo", ticks_per_second);
            telemetry.addData("Shooter Pronto", velocidade_MAX);
            telemetry.update();
            motor_in.setPower(0.4);

    }


    public void atualizarServoPorCor() {
        // Usa Sistema_visao.DetectedColor para garantir que o tipo seja reconhecido
        Sistema_visao.DetectedColor cor = vision.getDetectedColor();

        // Se for AZUL ou VERMELHO -> Posição 1.0. Caso contrário (Amarelo ou Nada) -> Posição 0.0
        if (cor == Sistema_visao.DetectedColor.BLUE || cor == Sistema_visao.DetectedColor.RED) {
            indexer.setPosition(1.0);
        } else {
            indexer.setPosition(0.0);
        }
    }
    // Função de encerramento
    public void stop() {
        vision.close();
    }

    public void sho_auto() throws InterruptedException {
        motor_sho.setPower(1);
        sleep(1500);
        motor_in.setPower(1);
        sleep(3000);
        motor_in.setPower(0);
        motor_sho.setPower(0);
    }
}

