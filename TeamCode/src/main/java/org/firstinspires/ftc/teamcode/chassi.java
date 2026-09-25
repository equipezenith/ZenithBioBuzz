package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.robotcore.hardware.IMU;

public class chassi {
    DcMotorEx motor_d_d; // motor dianteiro direito
    DcMotorEx motor_d_e; // motor dianteiro esquerdo
    DcMotorEx motor_t_d; // motor traseiro direito
    DcMotorEx motor_t_e; // motor traseiro esquerdo
    DcMotorEx xencoder;
    DcMotorEx yencoder;

    static double TICKS_PER_REV = 423.4;

    static double TICKS_PER_CM = 28/(Math.PI * 10.4); //TICKS_PER_REV = 28 / (Math.PI * WHEEL_DIAMETER_CM = 10.4)
    double theta;
    double x;
    private IMU imu;
    double angle;
    double y;
    double turn;
    double power;
    double cos;
    double sin;
    double max;
    int targetTicks;
    double x_position;
    double y_position;
    int x_p_position;
    int y_p_position;
    double front_a;
    int front_p;
    double error_x;
    double error_y;
    double error_front;
    double kp;
    double kp_turn;
    double FL_power;
    double FR_power;
    double BL_power;
    double BR_power;


    //Construtor do chassi serve para puxar o hardwareMap
    public chassi (HardwareMap hardwareMap) {
        motor_d_d = hardwareMap.get(DcMotorEx.class, "motordd"); //Motor CHASSI - Dianteiro Direito
        motor_d_e = hardwareMap.get(DcMotorEx.class, "motorde"); //Motor CHASSI - Dianteiro Esquerdo
        motor_t_d = hardwareMap.get(DcMotorEx.class, "motortd"); //Motor CHASSI - Traseiro Direito
        motor_t_e = hardwareMap.get(DcMotorEx.class, "motorte"); //Motor CHASSI - Traseiro Esquerdo
        imu = hardwareMap.get(IMU.class, "imu");
        xencoder = hardwareMap.get(DcMotorEx.class, "motordd");
        yencoder = hardwareMap.get(DcMotorEx.class, "motorde");


        motor_d_d.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_d_e.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_t_d.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_t_e.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RevHubOrientationOnRobot orientacaorobo = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT ,
                RevHubOrientationOnRobot.UsbFacingDirection.UP

        );
        imu.initialize(new IMU.Parameters(orientacaorobo));

        // Inverter motores do lado esquerdo (se necessário)
        motor_d_e.setDirection(DcMotor.Direction.FORWARD);
        motor_t_e.setDirection(DcMotor.Direction.FORWARD);
        motor_d_d.setDirection(DcMotor.Direction.REVERSE);
        motor_t_d.setDirection(DcMotor.Direction.REVERSE);
    }

    public void andarCM (double cm, double power) {

        targetTicks = (int) (cm * (TICKS_PER_CM * 15));



        motor_d_e.setTargetPosition(motor_d_e.getCurrentPosition() + (targetTicks));
        motor_d_d.setTargetPosition(motor_d_d.getCurrentPosition() + (targetTicks));
        motor_t_e.setTargetPosition(motor_t_e.getCurrentPosition() + (targetTicks));
        motor_t_d.setTargetPosition(motor_t_d.getCurrentPosition() + (targetTicks));


        motor_d_e.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_d_d.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_t_e.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_t_d.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motor_d_e.setPower(0.7);
        motor_d_d.setPower(power);
        motor_t_e.setPower(power);
        motor_t_d.setPower(0.7);

        while (motor_d_e.isBusy() || motor_d_d.isBusy() || motor_t_e.isBusy() || motor_t_d.isBusy()) {
        }

        motor_d_e.setPower(0);
        motor_d_d.setPower(0);
        motor_t_e.setPower(0);
        motor_t_d.setPower(0);

        motor_d_e.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_d_d.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_t_e.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_t_d.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
    public void andarDiagonal(double cm, double power, String direcao) {

        // Calcula os ticks baseados no valor absoluto (sempre positivo)
        int targetTicks = (int) (Math.abs(cm) * (TICKS_PER_CM * 15));

        int alvo_d_e = 0;
        int alvo_d_d = 0;
        int alvo_t_e = 0;
        int alvo_t_d = 0;

        // Define quais motores vão se mover baseado na direção
        switch (direcao.toUpperCase()) {
            case "FRENTE_DIREITA":
                alvo_d_e = targetTicks;
                alvo_t_d = targetTicks;
                break;
            case "TRAS_ESQUERDA":
                alvo_d_e = -targetTicks;
                alvo_t_d = -targetTicks;
                break;
            case "FRENTE_ESQUERDA":
                alvo_d_d = targetTicks;
                alvo_t_e = targetTicks;
                break;
            case "TRAS_DIREITA":
                alvo_d_d = -targetTicks;
                alvo_t_e = -targetTicks;
                break;
        }

        // Adiciona o alvo à posição atual de cada motor
        motor_d_e.setTargetPosition(motor_d_e.getCurrentPosition() + alvo_d_e);
        motor_d_d.setTargetPosition(motor_d_d.getCurrentPosition() + alvo_d_d);
        motor_t_e.setTargetPosition(motor_t_e.getCurrentPosition() + alvo_t_e);
        motor_t_d.setTargetPosition(motor_t_d.getCurrentPosition() + alvo_t_d);

        motor_d_e.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_d_d.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_t_e.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor_t_d.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Garante potência positiva e aplica apenas nos motores que vão se mover
        double absPower = Math.abs(power);

        // Se o alvo for 0, a potência enviada deve ser 0 para a roda ficar travada
        motor_d_e.setPower(alvo_d_e != 0 ? absPower : 0);
        motor_d_d.setPower(alvo_d_d != 0 ? absPower : 0);
        motor_t_e.setPower(alvo_t_e != 0 ? absPower : 0);
        motor_t_d.setPower(alvo_t_d != 0 ? absPower : 0);

        // Espera os motores terminarem
        while (motor_d_e.isBusy() || motor_d_d.isBusy() || motor_t_e.isBusy() || motor_t_d.isBusy()) {
        }

        // Para tudo ao final
        motor_d_e.setPower(0);
        motor_d_d.setPower(0);
        motor_t_e.setPower(0);
        motor_t_d.setPower(0);

        motor_d_e.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_d_d.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_t_e.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor_t_d.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void att_posicao () {
        //Cria as variaveis locais
        int curretXposition = xencoder.getCurrentPosition();
        int curretYposition = yencoder.getCurrentPosition();
        front_a = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        //Calcula a variação desde o ultimo ciclo
        double deltaX = (curretXposition - x_p_position) / TICKS_PER_CM;
        double deltaY = (curretYposition - y_p_position) / TICKS_PER_CM;

        double deltaXGlobal = deltaX * Math.cos(front_a) - deltaY * Math.sin(front_a);
        double deltaYGlobal = deltaX * Math.sin(front_a) + deltaY * Math.cos(front_a);

        x_position += deltaXGlobal;
        y_position += deltaYGlobal;

        x_p_position = curretXposition;
        y_p_position = curretYposition;

    }

    public void movOP(Gamepad gamepad1, Telemetry telemetry) {
        // Leitura dos joysticks
        x = gamepad1.left_stick_x;
        y = -gamepad1.left_stick_y; // invertido porque pra frente é negativo no FTC
        turn = gamepad1.right_stick_x;

        // Cálculo de ângulo e intensidade
        theta = Math.atan2(y, x);
        power = Math.hypot(x, y);

        // Variavel do limitador de RPM
        double ticks_per_second = (300/60.0) * TICKS_PER_REV;

        // Seno e cosseno com ajuste de 45°*/\

        sin = Math.sin(theta - Math.PI / 4);
        cos = Math.cos(theta - Math.PI / 4);
        max = Math.max(Math.abs(sin), Math.abs(cos));

        // Potência de cada roda
        double leftFront  = power * cos / max + turn;
        double rightFront = power * sin / max - turn;
        double leftRear   = power * sin / max + turn;
        double rightRear  = power * cos / max - turn;

        // Normalização (evita passar de 1)
        double maxPower = Math.max(
                Math.max(Math.abs(leftFront), Math.abs(rightFront)),
                Math.max(Math.abs(leftRear), Math.abs(rightRear))
        );

        if (maxPower > 1.0) {
            leftFront  /= maxPower;
            rightFront /= maxPower;
            leftRear   /= maxPower;
            rightRear  /= maxPower;
        }

        double velocidade;

        if (gamepad1.a) {
            velocidade = 0.3; // 30% da velocidade quando segura o A
        } else {
            velocidade = 1.0; // 100% da velocidade normal
        }

        // Envia potência pros motores
        motor_d_e.setVelocity(leftFront * velocidade * ticks_per_second);
        motor_d_d.setVelocity(rightFront * velocidade * ticks_per_second);
        motor_t_e.setVelocity(leftRear * velocidade * ticks_per_second);
        motor_t_d.setVelocity(rightRear * velocidade * ticks_per_second);



        //Telemetria (pra debug)
//            telemetry.addData("LF", leftFront);
//            telemetry.addData("RF", rightFront);
//            telemetry.addData("LR", leftRear);
//            telemetry.addData("RR", rightRear);
//            telemetry.update();
    }



    public void girarGraus (double graus, double vel) {
        imu.resetYaw();
        angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        if (graus > 0) {
            motor_d_d.setPower(vel);
            motor_t_d.setPower(vel);
            motor_d_e.setPower(-vel);
            motor_t_e.setPower(-vel);
            while (graus > angle ) {
                angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            }
            if (graus != angle) {
                motor_d_d.setPower(-0.2);
                motor_t_d.setPower(-0.2);
                motor_d_e.setPower(0.2);
                motor_t_e.setPower(0.2);
                while (graus < angle ) {
                    angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                }
            }
        } else if (graus < 0){
            motor_d_d.setPower(-vel);
            motor_t_d.setPower(-vel);
            motor_d_e.setPower(vel);
            motor_t_e.setPower(vel);
            while (graus < angle ) {
                angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            }
            if (graus != angle) {
                motor_d_d.setPower(0.2);
                motor_t_d.setPower(0.2);
                motor_d_e.setPower(-0.2);
                motor_t_e.setPower(-0.2);
                while (graus > angle ) {
                    angle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                }
            }
        }

        while (motor_d_e.isBusy() || motor_d_d.isBusy() || motor_t_e.isBusy() || motor_t_d.isBusy()) {
        }

        motor_d_d.setPower(0);
        motor_t_d.setPower(0);
        motor_d_e.setPower(0);
        motor_t_e.setPower(0);

    }
}