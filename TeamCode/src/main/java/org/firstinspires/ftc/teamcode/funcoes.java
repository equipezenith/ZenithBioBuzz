package org.firstinspires.ftc.teamcode;
import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class
funcoes extends chassi  {

    // Variável global (pode ser setada de qualquer classe: Robo.novoCiclo = true;)
    boolean novoCiclo = true;   // true = pode ativar na primeira vez

    // Campos da classe
    private enum Estado { ARMADO, SEGURANDO, TRAVADO }
    private Estado estado = Estado.ARMADO;
    private final ElapsedTime holdTimer = new ElapsedTime();
    private static final double TEMPO_MINIMO_SEG = 1.0;

    int atag0;
    int atag1;
    int atag2;
    private VisionPortal visionPortal;
    private AprilTagProcessor tagProcessor;
    public Sistema_visao vision_cor = new Sistema_visao();

    DcMotorEx motor_in;

    DcMotorEx motor_tra;

    public boolean in_state = false;

    DcMotorEx motor_sho;
    Servo indexerN;

    Servo indexerP;

    public funcoes (HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, new Pose(9 ,9 , Math.toRadians(90)));
        motor_sho =  hardwareMap.get(DcMotorEx.class, "motorSho");
        motor_sho.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motor_tra =  hardwareMap.get(DcMotorEx.class, "motorTra");
        motor_tra.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_in = hardwareMap.get(DcMotorEx.class, "motorIn");
        indexerP = hardwareMap.get(Servo.class, "indexerP");
        indexerP.setPosition(0.0);
        indexerN = hardwareMap.get(Servo.class, "indexerN");
        indexerN.setPosition(0.0);
        vision_cor.init(hardwareMap, "CAMERA_COR", Sistema_visao.Mode.COLOR, viewIds[0]);

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

    public void shooterOPPolem(Telemetry telemetry){
        double kS = 0.262, kV = 0.000227, kP = 0.0052;
        indexerN.setPosition(0);
        indexerP.setPosition(0);
        double goalRPM = 1700;
        double goalTicks = (goalRPM * 28) / 60;
        boolean goalReached = false;
        double feedForward = (kV * goalRPM) + kS;
        double error = goalRPM - (motor_sho.getVelocity() * 60) / 28;
        double feedBack = error * kP;
        double powerP = feedBack + feedForward;
        novoCiclo = true;

        motor_sho.setPower(powerP);
        if (goalTicks - 9.5 >= motor_sho.getVelocity()){
            goalReached = true;
        }
        if (motor_sho.getVelocity() >= goalTicks - 9.5 || (goalReached && motor_sho.getVelocity() >= goalTicks - 50)) {
            motor_tra.setPower(1);
        } else {
            motor_tra.setPower(0);
        }

        telemetry.addData("error", error );
        telemetry.addData("RPM Atual", (motor_sho.getVelocity() * 60) / 28);
        telemetry.addData("Shooter pronto?:", goalReached);

    }

    public void shooterOPNectar(Telemetry telemetry){
        indexerN.setPosition(0);
        indexerP.setPosition(0);
        double kS = 0.302, kV = 0.000171,kP = 0.006;
        double goalRPM = 1928;
        double goalTicks = (goalRPM * 28) / 60;
        double RPMatual = (motor_sho.getVelocity() * 60) / 28;
        boolean goalReached = false;
        double feedForward = (kV * goalRPM) + kS;
        double error = goalRPM - (motor_sho.getVelocity() * 60) / 28;
        double feedBack = error * kP;
        double powerN = feedBack + feedForward;
        novoCiclo = true;
        motor_sho.setPower(powerN);
        if (RPMatual >= goalRPM){
            goalReached = true;
        }
        if (motor_sho.getVelocity() >= goalTicks) {
            motor_tra.setPower(1);
        } else {
            motor_tra.setPower(0);
        }

        telemetry.addData("error", error );
        telemetry.addData("RPM Atual", (motor_sho.getVelocity() * 60) / 28);
        telemetry.addData("Shooter pronto?:", goalReached);

    }



    public void atualizarServoPorCor(boolean ativo) {
        Servo servoAtivo = null;
        Sistema_visao.DetectedColor cor = vision_cor.getDetectedColor();
        boolean nectarDetectado = (cor == Sistema_visao.DetectedColor.BLUE
                || cor == Sistema_visao.DetectedColor.RED);
        boolean polenDetectado = (cor == Sistema_visao.DetectedColor.YELLOW);

        switch (estado) {
            case ARMADO:
                if (ativo && novoCiclo && (nectarDetectado || polenDetectado)) {
                    novoCiclo = false;
                    servoAtivo = nectarDetectado ? indexerN : indexerP;
                    servoAtivo.setPosition(1.0);
                    holdTimer.reset();
                    estado = Estado.SEGURANDO;
                }
                break;

            case SEGURANDO:
                if (holdTimer.seconds() >= TEMPO_MINIMO_SEG) {
                    servoAtivo.setPosition(0.0);
                    servoAtivo = null;
                    estado = Estado.TRAVADO;
                }
                break;

            case TRAVADO:
                if (novoCiclo) estado = Estado.ARMADO;
                break;
        }
    }

    // Função de encerramento
    public void stop() {
        vision_cor.close();
    }

    public void sho_auto() throws InterruptedException {
        motor_sho.setPower(1);
        sleep(1000);
        motor_tra.setPower(0.4);
    }
}

