package org.firstinspires.ftc.teamcode;

import android.app.Activity;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "OpPrototipos")
public class Op_prototipos extends OpMode {
    DcMotor motor;
    DcMotorEx motor_d_d; // motor dianteiro direito
    DcMotorEx motor_d_e; // motor dianteiro esquerdo
    DcMotorEx motor_t_d; // motor traseiro direito
    DcMotorEx motor_t_e; // motor traseiro esquerdo


    private ElapsedTime timer = new ElapsedTime();

    private final java.util.concurrent.ExecutorService redeExecutor = java.util.concurrent.Executors.newSingleThreadExecutor();

    @Override
    public void init() {

        motor =  hardwareMap.get(DcMotor.class, "motor");
        motor_d_d = hardwareMap.get(DcMotorEx.class, "motordd"); //Motor CHASSI - Dianteiro Direito
        motor_d_e = hardwareMap.get(DcMotorEx.class, "motorde"); //Motor CHASSI - Dianteiro Esquerdo
        motor_t_d = hardwareMap.get(DcMotorEx.class, "motortd"); //Motor CHASSI - Traseiro Direito
        motor_t_e = hardwareMap.get(DcMotorEx.class, "motorte"); //Motor CHASSI - Traseiro Esquerdo
    }

    @Override
    public void loop() {
        if (gamepad2.left_trigger > 0) {
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
            motor.setPower(1);
        } else {
            motor.setPower(0);
        }
        if (gamepad2.left_bumper) {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
            motor.setPower(1);
        } else {
            motor.setPower(0);
        }


    }
}