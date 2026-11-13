package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import java.util.Arrays;

// Ajuste para o pacote onde o Pedro 3 gerou a classe Constants no seu projeto
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Gamepad1:
 *   A         -> vai ao arco mais próximo (superior ou inferior) e alinha com a Limelight
 *   D-pad ←   -> reset no canto superior esquerdo
 *   D-pad →   -> reset no canto inferior esquerdo
 * Giro: PD no tx da tag. Distância: horizontal, calculada pelo ty + geometria da montagem.
 */
@Config
@TeleOp(name = "Ir ao Arco + Alinhar (Limelight)", group = "Teste")
public class IrAoArco extends OpMode {

    // Pose de partida: canto inferior esquerdo
    public static double START_X = 9, START_Y = 9, START_HEADING_DEG = 90;

    // Resets nos cantos
    public static double RESET_BL_X = 9, RESET_BL_Y = 9, RESET_BL_HEADING_DEG = 90;
    public static double RESET_TL_X = 9, RESET_TL_Y = 135, RESET_TL_HEADING_DEG = 90;

    // ===================== LIMELIGHT =====================
    public static String LIMELIGHT_NAME = "limelight";
    public static int PIPELINE_TAG = 0;
    public static int POLL_RATE_HZ = 100;
    public static double CAMERA_HEIGHT_MM = 262.88; // altura da lente no chão
    public static double CAMERA_ANGLE_DEG = 49;     // + = câmera inclinada para cima
    public static double TAG_HEIGHT_MM = 1261;      // altura do centro da tag

    private static final double MM_TO_IN = 1.0 / 25.4;

    // Distância horizontal desejada até a tag, na posição de tiro (polegadas)
    public static double DESIRED_RANGE_IN = 30;
    public static double RANGE_TOLERANCE_IN = 1.0;
    public static double kRange = 0.03;
    public static double MAX_DRIVE = 0.4;

    // Giro (PD)
    public static double GOAL_X = 0;               // tx desejado (graus)
    public static double ANGLE_TOLERANCE = 0.5;    // graus
    public static double kP = 0.002;
    public static double kD = 0.0001;
    public static double MAX_ROTATE = 0.4;
    public static double CAMERA_TURN_SIGN = 1;     // -1 se girar para o lado errado
    public static double kHeadingP = 0.015;

    public static double ALIGN_TIMEOUT_S = 5;

    // ===================== TAGS DAS DUAS FACES DO HIVE =====================
    // Arco inferior -> face inferior. Arco superior -> face superior.
    // PLACEHOLDER no superior: confirme os IDs da face de cima.
    private static final int[] TAGS_FACE_INFERIOR = {42, 43, 44, 45};  // azul, por enquanto
    private static final int[] TAGS_FACE_SUPERIOR = {38, 39, 40, 41};  // placeholder

    private enum State { MANUAL, DRIVE_TO_ARC, ALIGN }

    private Follower follower;
    private Limelight3A limelight;

    private State state = State.MANUAL;
    private Pose arcTarget = null;
    private int[] tagsAlinhamento = TAGS_FACE_INFERIOR;
    private String faceEscolhida = "inferior";
    private boolean superiorAlvo = false; // D-pad cima = superior, D-pad baixo = inferior
    private boolean upPrev = false;
    private boolean downPrev = false;
    private String ultimoReset = "nenhum";

    private boolean aPrev = false;
    private boolean leftPrev = false;
    private boolean rightPrev = false;

    private double lastError = 0;
    private double lastTime = 0;
    private double alignStart = 0;
    private boolean hasLastError = false;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG)));

        limelight = hardwareMap.get(Limelight3A.class, LIMELIGHT_NAME);
        limelight.pipelineSwitch(PIPELINE_TAG);
        limelight.setPollRateHz(POLL_RATE_HZ);
        limelight.start(); // sem start() os resultados nunca são válidos

        telemetry.addLine("Pronto. Canto inferior esquerdo. A = ir ao arco e alinhar.");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        Pose pose = follower.pose();

        // Tag do lado ativo vista agora (uma leitura por loop)
        LLResultTypes.FiducialResult tagVista = melhorFiducial(tagsAlinhamento);

        // D-pad cima/baixo: escolhe o arco-alvo (superior / inferior)
        boolean upNow = gamepad1.dpad_up;
        boolean downNow = gamepad1.dpad_down;
        if (upNow && !upPrev) superiorAlvo = true;
        if (downNow && !downPrev) superiorAlvo = false;
        upPrev = upNow;
        downPrev = downNow;

        // Resets nos cantos: só parado e no manual
        boolean leftNow = gamepad1.dpad_left;
        boolean rightNow = gamepad1.dpad_right;
        if (state == State.MANUAL && !follower.isBusy()) {
            if (leftNow && !leftPrev) {
                resetPose(RESET_TL_X, RESET_TL_Y, RESET_TL_HEADING_DEG);
                ultimoReset = "canto superior esquerdo";
            }
            if (rightNow && !rightPrev) {
                resetPose(RESET_BL_X, RESET_BL_Y, RESET_BL_HEADING_DEG);
                ultimoReset = "canto inferior esquerdo";
            }
        }
        leftPrev = leftNow;
        rightPrev = rightNow;

        // A: só a partir do manual
        boolean aNow = gamepad1.a;
        if (aNow && !aPrev && state == State.MANUAL) {
            arcTarget = ArcoHive.nearestPoseOnArc(pose, superiorAlvo);
            if (ArcoHive.distance(pose, arcTarget) > ArcoHive.ON_ARC_TOLERANCE) {
                follower.follow(ArcoHive.buildPath(pose, arcTarget)); // confira o nome na sua versão
                state = State.DRIVE_TO_ARC;
            } else {
                startAlign();
                state = State.ALIGN;
            }
        }
        aPrev = aNow;

        double forward = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        boolean pedroDrivingPath = false;

        switch (state) {
            case MANUAL:
                break;

            case DRIVE_TO_ARC:
                pedroDrivingPath = true;
                if (!follower.isBusy()) {
                    startAlign();
                    state = State.ALIGN;
                }
                break;

            case ALIGN: {
                double now = getRuntime();
                double dt = now - lastTime;
                lastTime = now;

                forward = 0;
                rotate = 0;

                if (now - alignStart > ALIGN_TIMEOUT_S) {
                    state = State.MANUAL;
                } else if (tagVista != null) {
                    double bearingErr = GOAL_X - tagVista.getTargetXDegrees();
                    double rangeErr = distanciaHorizontalIn(tagVista) - DESIRED_RANGE_IN;

                    boolean rotOk = Math.abs(bearingErr) < ANGLE_TOLERANCE;
                    boolean rangeOk = Math.abs(rangeErr) < RANGE_TOLERANCE_IN;

                    if (rotOk && rangeOk) {
                        state = State.MANUAL;
                        hasLastError = false;
                    } else {
                        double derivative = (hasLastError && dt > 0) ? (bearingErr - lastError) / dt : 0;
                        lastError = bearingErr;
                        hasLastError = true;

                        rotate = CAMERA_TURN_SIGN * Range.clip(bearingErr * kP + derivative * kD, -MAX_ROTATE, MAX_ROTATE);
                        // distância maior que o alvo = longe demais = anda para frente
                        forward = rangeOk ? 0 : Range.clip(rangeErr * kRange, -MAX_DRIVE, MAX_DRIVE);
                    }
                } else {
                    // Tag fora de vista: gira pela odometria em direção ao hive
                    hasLastError = false;
                    double desired = ArcoHive.headingToHive(pose.x(), pose.y())
                            + Math.toRadians(ArcoHive.HEADING_OFFSET_DEG);
                    double hErr = ArcoHive.normalize180(Math.toDegrees(desired - pose.heading()));
                    rotate = Range.clip(hErr * kHeadingP, -MAX_ROTATE, MAX_ROTATE);
                }
                break;
            }
        }

        if (!pedroDrivingPath) {
            follower.manual(forward, lateral, rotate);
        }

        telemetry.addData("Estado", state);
        telemetry.addData("Pose", "x %.1f  y %.1f  h %.1f°",
                pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        telemetry.addData("Último reset", ultimoReset);
        telemetry.addData("Alvo (D-pad ↑/↓)", superiorAlvo ? "superior" : "inferior");
        telemetry.addData("Face usada", faceEscolhida);
        telemetry.addData("Tags procuradas", Arrays.toString(tagsAlinhamento));
        if (arcTarget != null) {
            telemetry.addData("Alvo no arco", "x %.1f  y %.1f", arcTarget.x(), arcTarget.y());
        }
        if (tagVista != null) {
            telemetry.addData("Tag vista", tagVista.getFiducialId());
            telemetry.addData("Distância horizontal", "%.1f in (alvo %.1f)",
                    distanciaHorizontalIn(tagVista), DESIRED_RANGE_IN);
            telemetry.addData("tx", "%.2f°", tagVista.getTargetXDegrees());
        } else {
            telemetry.addLine("Nenhuma tag do lado ativo visível");
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        limelight.stop();
    }

    // ===================== Limelight =====================

    /** Distância horizontal (chão) até a tag, em polegadas, a partir do ty. */
    private double distanciaHorizontalIn(LLResultTypes.FiducialResult f) {
        double angulo = Math.toRadians(CAMERA_ANGLE_DEG + f.getTargetYDegrees());
        double deltaH = (TAG_HEIGHT_MM - CAMERA_HEIGHT_MM) * MM_TO_IN;
        return deltaH / Math.tan(angulo);
    }

    /** Entre as tags visíveis da lista, devolve a mais centralizada (menor |tx|). */
    private LLResultTypes.FiducialResult melhorFiducial(int[] ids) {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        LLResultTypes.FiducialResult best = null;
        for (LLResultTypes.FiducialResult f : result.getFiducialResults()) {
            if (!contem(ids, f.getFiducialId())) continue;
            if (best == null || Math.abs(f.getTargetXDegrees()) < Math.abs(best.getTargetXDegrees())) {
                best = f;
            }
        }
        return best;
    }

    private static boolean contem(int[] ids, int id) {
        for (int v : ids) {
            if (v == id) return true;
        }
        return false;
    }

    // ===================== estado =====================

    private void resetPose(double x, double y, double headingDeg) {
        follower.setPose(new Pose(x, y, Math.toRadians(headingDeg)));
    }

    private void startAlign() {
        // A face é escolhida pelo arco em que o robô está e fica travada até o fim do alinhamento
        boolean superior = superiorAlvo;
        tagsAlinhamento = superior ? TAGS_FACE_SUPERIOR : TAGS_FACE_INFERIOR;
        faceEscolhida = superior ? "superior (arco de cima)" : "inferior (arco de baixo)";

        hasLastError = false;
        lastTime = getRuntime();
        alignStart = getRuntime();
    }
}