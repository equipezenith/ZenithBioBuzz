package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

/**
 * Auto-alinhamento com AprilTag + mecanum, baseado no tutorial do Brogan M. Pratt.
 * Segurando LEFT TRIGGER (> 0.3), o giro (rotate) é controlado por um PD que centraliza a tag;
 * frente e lateral continuam nos manches.
 *
 * Câmeras:  CAMERA_COR = indexer (cor)  |  CAMERA_TAG = AprilTag
 *
 * Ajuste ao vivo pelo gamepad1 (igual ao vídeo):
 *   B                -> troca o tamanho do passo
 *   D-pad esq / dir  -> diminui / aumenta KP
 *   D-pad baixo / cima -> diminui / aumenta KD
 */
@Config
@TeleOp(name = "Mecanum + 2 Cameras", group = "Teste")
public class AutoAlignTeleOp extends LinearOpMode {

    // Nomes na configuração do robô (ajuste para os seus)
    public static String FL_NAME = "leftFront";
    public static String FR_NAME = "rightFront";
    public static String BL_NAME = "leftBack";
    public static String BR_NAME = "rightBack";

    public static String CAMERA_COR = "Webcam 1";
    public static String CAMERA_TAG = "Webcam 2";

    // ------------------------- variáveis do PD (tutorial) -------------------------
    public static int TARGET_TAG_ID = 39;      // precisa estar entre TAG_ID_MIN e TAG_ID_MAX do Sistema_visao
    public static double kP = 0.002;           // começa bem baixo
    public static double kD = 0.0001;          // ainda mais baixo que o P
    public static double goalX = 0;            // bearing desejado (pode ser um offset, ex.: -2, 0.5)
    public static double angleTolerance = 0.2; // graus; webcam menos precisa => tolerância maior
    public static double MAX_ROTATE = 0.4;     // limite do giro (clamp -0.4 a 0.4)

    private double error = 0;
    private double lastError = 0;
    private double curTime = 0;
    private double lastTime = 0;

    // ------------------------- ajuste ao vivo pelo controle -------------------------
    private final double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001};
    private int stepIndex = 2;
    private boolean bPrev, leftPrev, rightPrev, upPrev, downPrev;

    private DcMotor leftFront, rightFront, leftBack, rightBack;

    private final Sistema_visao visaoCor = new Sistema_visao(); // câmera do indexer
    private final Sistema_visao visaoTag = new Sistema_visao(); // câmera da AprilTag

    @Override
    public void runOpMode() {
        leftFront = hardwareMap.get(DcMotor.class, FL_NAME);
        rightFront = hardwareMap.get(DcMotor.class, FR_NAME);
        leftBack = hardwareMap.get(DcMotor.class, BL_NAME);
        rightBack = hardwareMap.get(DcMotor.class, BR_NAME);

        // Direções; inverta se o seu robô andar ao contrário
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{leftFront, rightFront, leftBack, rightBack}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Com duas câmeras, a tela do Driver Station precisa ser dividida (senão a 2ª falha)
        int[] viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.VERTICAL);

        visaoCor.init(hardwareMap, CAMERA_COR, Sistema_visao.Mode.COLOR, viewIds[0]);
        visaoTag.init(hardwareMap, CAMERA_TAG, Sistema_visao.Mode.APRILTAG, viewIds[1]);

        // Exposição baixa + ganho alto na câmera da tag (dica do vídeo: loop mais rápido)
        visaoTag.setManualExposure();

        telemetry.addLine("Initialized. Segure LEFT TRIGGER para alinhar na AprilTag.");
        telemetry.update();

        waitForStart();

        // Zera o relógio para o dt do derivativo (equivale ao start() do vídeo)
        resetRuntime();
        curTime = getRuntime();
        lastTime = curTime;

        while (opModeIsActive()) {
            // ------------------------- entradas do mecanum -------------------------
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double rotate = gamepad1.right_stick_x;

            // ------------------------- info da AprilTag -------------------------
            // (substitui aprilTagWebcam.update() + getTagBySpecificId(20))
            AprilTagSingleDetection id39 = visaoTag.getDetectionById(TARGET_TAG_ID);

            // Cor lida pela câmera do indexer (sempre ativa)
            Sistema_visao.DetectedColor cor = visaoCor.getDetectedColor();

            // ------------------------- auto align rotation logic -------------------------
            if (gamepad1.left_trigger > 0.3) {
                if (id39 != null) {
                    error = goalX - id39.ftcPose.bearing; // tx

                    if (Math.abs(error) < angleTolerance) {
                        rotate = 0;
                    } else {
                        double pTerm = error * kP;

                        curTime = getRuntime();
                        double dt = curTime - lastTime;
                        double dTerm = (dt > 0) ? ((error - lastError) / dt) * kD : 0;

                        rotate = Range.clip(pTerm + dTerm, -MAX_ROTATE, MAX_ROTATE);

                        lastError = error;
                        lastTime = curTime;
                    }
                } else {
                    // Tag errada/ausente: evita pico de derivada quando ela voltar
                    lastTime = getRuntime();
                    lastError = 0;
                }
            } else {
                // Gatilho solto: mesmo motivo
                lastTime = getRuntime();
                lastError = 0;
            }

            // ------------------------- ajuste do PD pelo controle -------------------------
            boolean b = gamepad1.b;
            boolean left = gamepad1.dpad_left, right = gamepad1.dpad_right;
            boolean up = gamepad1.dpad_up, down = gamepad1.dpad_down;

            if (b && !bPrev) stepIndex = (stepIndex + 1) % stepSizes.length;
            if (left && !leftPrev) kP -= stepSizes[stepIndex];
            if (right && !rightPrev) kP += stepSizes[stepIndex];
            if (up && !upPrev) kD += stepSizes[stepIndex];
            if (down && !downPrev) kD -= stepSizes[stepIndex];

            bPrev = b; leftPrev = left; rightPrev = right; upPrev = up; downPrev = down;

            // ------------------------- telemetria -------------------------
            if (id39 != null) {
                if (gamepad1.left_trigger > 0.3) {
                    telemetry.addLine("AUTO ALIGN");
                } else {
                    telemetry.addLine("Rotação manual");
                }
                telemetry.addData("Bearing", "%.2f", id39.ftcPose.bearing);
                telemetry.addData("Error", "%.2f", error);
            } else {
                telemetry.addLine("Tag " + TARGET_TAG_ID + " não visível (rotação manual)");
            }
            telemetry.addData("KP", "%.5f", kP);
            telemetry.addData("KD", "%.5f", kD);
            telemetry.addData("Step", "%.5f", stepSizes[stepIndex]);
            telemetry.addData("Tags vistas", visaoTag.getDetectedIds());
            telemetry.addData("Cor (indexer)", cor);
            telemetry.addData("Pixels R/B", "%d / %d", visaoCor.getRedPixels(), visaoCor.getBluePixels());

            // ------------------------- drive -------------------------
            moveRobot(forward, strafe, rotate);
            telemetry.update();
        }

        visaoCor.close();
        visaoTag.close();
    }

    // Mecanum padrão: forward (+ = frente), strafe (+ = direita), rotate (+ = horário)
    private void moveRobot(double forward, double strafe, double rotate) {
        double lf = forward + strafe + rotate;
        double rf = forward - strafe - rotate;
        double lb = forward - strafe + rotate;
        double rb = forward + strafe - rotate;

        double max = Math.max(1.0, Math.max(Math.abs(lf), Math.max(Math.abs(rf),
                Math.max(Math.abs(lb), Math.abs(rb)))));

        leftFront.setPower(lf / max);
        rightFront.setPower(rf / max);
        leftBack.setPower(lb / max);
        rightBack.setPower(rb / max);
    }
}