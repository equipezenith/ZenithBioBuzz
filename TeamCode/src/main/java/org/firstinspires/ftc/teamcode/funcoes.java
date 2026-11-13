package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class funcoes extends chassi {

    // Máquina de estados do Indexer
    public enum Estado { ARMADO, SEGURANDO, TRAVADO }
    private Estado estado = Estado.ARMADO;

    public boolean novoCiclo = true;
    private Servo servoAtivo = null;
    private final ElapsedTime holdTimer = new ElapsedTime();
    private static final double TEMPO_MINIMO_SEG = 1.0;
    private boolean shooterPPronto = false;
    private boolean shooterNPronto = false;
    // Configuração das posições dos servos
    public static final double POS_NECTAR_ABERTO = 1.0; // Abre 180° (1.0)
    public static final double POS_POLEN_ABERTO  = 0.5; // Abre 90°  (0.5)
    public static final double POS_FECHADO       = 0.0; // Posição fechada (0.0)

    private double posAbertoAtual = POS_FECHADO; // Posição do ciclo ativo

    // Sistema de Visão
    public Sistema_visao vision_cor = new Sistema_visao();

    // Hardware
    DcMotorEx motor_in;
    DcMotorEx motor_tra;
    DcMotorEx motor_shoP;
    DcMotorEx motor_shoN;
    Servo indexerN;
    Servo indexerP;

    public boolean in_state = false;
    public Telemetry telemetry;

    public funcoes(HardwareMap hardwareMap, Telemetry telemetry) {
        super(hardwareMap, new Pose(9, 9, Math.toRadians(90)));
        this.telemetry = telemetry;

        motor_shoP = hardwareMap.get(DcMotorEx.class, "motorShoP");
        motor_shoP.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_shoN = hardwareMap.get(DcMotorEx.class, "motorShoN");

        motor_tra = hardwareMap.get(DcMotorEx.class, "motorTra");
        motor_tra.setDirection(DcMotorSimple.Direction.REVERSE);
        motor_in = hardwareMap.get(DcMotorEx.class, "motorIn");

        indexerP = hardwareMap.get(Servo.class, "indexerP");
        indexerP.setPosition(1.0);
        indexerN = hardwareMap.get(Servo.class, "indexerN");
        indexerN.setPosition(1.0);

        // Inicialização da visão com ID de tela padrão (0)
        vision_cor.init(hardwareMap, "CAMERA_COR", Sistema_visao.Mode.COLOR, 0);
    }

    public void intake_Off(Gamepad gamepad, Telemetry telemetry) {
        if (gamepad.right_bumper) {
            motor_in.setDirection(DcMotorSimple.Direction.REVERSE);
            motor_in.setPower(1);
            in_state = true;
        } else {
            motor_in.setPower(0);
            in_state = false;
        }
    }

    public void intake_Off_inv(Gamepad gamepad, Telemetry telemetry) {
        if (gamepad.left_bumper) {
            motor_in.setPower(1);
            in_state = true;
        } else {
            motor_in.setPower(0);
            in_state = false;
        }
    }

    public void shooterOPPolem(Telemetry telemetry) {
        double kS = 0.262, kV = 0.000227, kP = 0.0052;
        double goalRPM = 1700;
        indexerN.setPosition(0);
        indexerP.setPosition(0);

        double rpmAtual = (motor_shoP.getVelocity() * 60) / 28;
        double error = goalRPM - rpmAtual;
        double power = kS + (kV * goalRPM) + (kP * error);
        motor_shoP.setPower(Range.clip(power, 0, 1));

        // Histerese: liga perto da meta, só desliga se cair bastante
        if (rpmAtual >= goalRPM - 20) shooterPPronto = true;
        else if (rpmAtual < goalRPM - 120) shooterPPronto = false;

        motor_tra.setPower(shooterPPronto ? 1 : 0);

        telemetry.addData("RPM P", rpmAtual);
        telemetry.addData("P pronto?", shooterPPronto);
    }

    public void shooterOPNectar(Telemetry telemetry) {
        double kS = 0.302, kV = 0.000171, kP = 0.006;
        double goalRPM = 1928;
        indexerN.setPosition(1);
        indexerP.setPosition(1);

        double rpmAtual = (motor_shoN.getVelocity() * 60) / 28;
        double error = goalRPM - rpmAtual;
        double power = kS + (kV * goalRPM) + (kP * error);
        motor_shoN.setPower(Range.clip(power, 0, 1));

        if (rpmAtual >= goalRPM - 20) shooterNPronto = true;
        else if (rpmAtual < goalRPM - 120) shooterNPronto = false;

        motor_tra.setPower(shooterNPronto ? 1 : 0);

        telemetry.addData("RPM N", rpmAtual);
        telemetry.addData("N velocity bruta", motor_shoN.getVelocity());
        telemetry.addData("N pronto?", shooterNPronto);
    }

    public void pararShooters() {
        motor_shoN.setPower(0);
        motor_shoP.setPower(0);
        motor_tra.setPower(0);
        shooterNPronto = false;
        shooterPPronto = false;
    }

    public void atualizarServoPorCor(boolean ativo) {
        Sistema_visao.DetectedColor cor = vision_cor.getDetectedColor();
        boolean nectarDetectado = (cor == Sistema_visao.DetectedColor.BLUE || cor == Sistema_visao.DetectedColor.RED);
        boolean polenDetectado = (cor == Sistema_visao.DetectedColor.YELLOW);

        switch (estado) {
            case ARMADO:
                if (ativo && novoCiclo && (nectarDetectado || polenDetectado)) {
                    novoCiclo = false;

                    // Seleciona o servo e a amplitude de abertura desejada
                    if (nectarDetectado) {
                        servoAtivo = indexerN;
                        posAbertoAtual = POS_NECTAR_ABERTO; // 1.0
                    } else {
                        servoAtivo = indexerP;
                        posAbertoAtual = POS_POLEN_ABERTO;  // 0.5
                    }

                    servoAtivo.setPosition(posAbertoAtual);
                    holdTimer.reset();
                    estado = Estado.SEGURANDO;
                }
                break;

            case SEGURANDO:
                if (holdTimer.seconds() >= TEMPO_MINIMO_SEG) {
                    if (servoAtivo != null) {
                        servoAtivo.setPosition(POS_FECHADO); // Retorna a 0.0
                        servoAtivo = null;
                    }
                    estado = Estado.TRAVADO;
                }
                break;

            case TRAVADO:
                // Re-arma quando o elemento sai do campo de visão da câmera
                if (!nectarDetectado && !polenDetectado) {
                    novoCiclo = true;
                    estado = Estado.ARMADO;
                }
                break;
        }
    }

    public Estado getEstadoAtual() {
        return estado;
    }

    public void stop() {
        vision_cor.close();
    }
}