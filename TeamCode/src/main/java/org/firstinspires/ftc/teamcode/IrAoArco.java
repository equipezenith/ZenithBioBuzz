package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.Arrays;

// Ajuste para o pacote onde o Pedro 3 gerou a classe Constants no seu projeto
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Gamepad1:
 *   A         -> vai ao ponto do arco mais perto e depois alinha com a câmera
 *   D-pad ←   -> reset de posição no canto superior esquerdo
 *   D-pad →   -> reset de posição no canto inferior esquerdo
 * Alinhamento: giro pelo bearing da tag, distância pelo ftcPose.range (sem FMAP).
 * Parado, os manches dirigem normalmente.
 */
@Config
@TeleOp(name = "Ir ao Arco + Alinhar (câmera)", group = "Teste")
public class IrAoArco extends OpMode {

    // Pose de partida: canto inferior esquerdo, robô encostado nas paredes, virado para cima
    public static double START_X = 9, START_Y = 9, START_HEADING_DEG = 90;

    // Reset de posição nos cantos (centro ~9 in da parede para um robô de ~18 in)
    public static double RESET_BL_X = 9, RESET_BL_Y = 9, RESET_BL_HEADING_DEG = 90;   // canto inferior esquerdo
    public static double RESET_TL_X = 9, RESET_TL_Y = 135, RESET_TL_HEADING_DEG = 90; // canto superior esquerdo

    // Câmera
    public static String CAMERA_TAG = "Webcam 2";

    // ===================== TAGS DAS DUAS FACES DO HIVE (gangorra) =====================
    // A face (e as tags) depende do ARCO escolhido: arco inferior -> face inferior, arco superior -> face superior.
    // PLACEHOLDER no superior: confirme os IDs da face de cima.
    private static final int[] TAGS_FACE_INFERIOR = {42, 43, 44, 45};  // arco de baixo (azul, por enquanto)
    private static final int[] TAGS_FACE_SUPERIOR = {38, 39, 40, 41};  // arco de cima (placeholder)

    // Distância: da LENTE da câmera até a face da tag, na posição de tiro (polegadas). MEÇA.
    public static double DESIRED_RANGE_IN = 30;
    public static double RANGE_TOLERANCE_IN = 1.0;
    public static double kRange = 0.03;
    public static double MAX_DRIVE = 0.4;

    // Giro (PD do tutorial)
    public static double GOAL_X = 0;
    public static double ANGLE_TOLERANCE = 0.5;
    public static double kP = 0.002;
    public static double kD = 0.0001;
    public static double MAX_ROTATE = 0.4;
    public static double CAMERA_TURN_SIGN = 1;      // -1 se girar para o lado errado
    public static double kHeadingP = 0.015;         // fallback pela odometria quando a tag some

    public static double ALIGN_TIMEOUT_S = 5;

    private enum State { MANUAL, DRIVE_TO_ARC, ALIGN }

    private Follower follower;
    private final Sistema_visao visaoTag = new Sistema_visao();

    private State state = State.MANUAL;
    private Pose arcTarget = null;
    private String ultimoReset = "nenhum";

    private boolean aPrev = false;
    private boolean leftPrev = false;
    private boolean rightPrev = false;

    private int[] tagsAlinhamento = TAGS_FACE_INFERIOR; // travada quando o alinhamento começa
    private String faceEscolhida = "inferior";

    private double lastError = 0;
    private double lastTime = 0;
    private double alignStart = 0;
    private boolean hasLastError = false;

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
        follower.setPose(new Pose(START_X, START_Y, Math.toRadians(START_HEADING_DEG)));

        // A câmera começa a transmitir AQUI (no INIT), não na chegada ao arco
        int[] viewIds = VisionPortal.makeMultiPortalView(1, VisionPortal.MultiPortalLayout.VERTICAL);
        visaoTag.init(hardwareMap, CAMERA_TAG, Sistema_visao.Mode.APRILTAG, viewIds[0]);
        visaoTag.setManualExposure();

        telemetry.addLine("Pronto. Canto inferior esquerdo. A = ir ao arco e alinhar.");
        telemetry.update();
    }

    @Override
    public void loop() {
        follower.update();
        Pose pose = follower.pose();

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
            arcTarget = ArcoHive.nearestArcPose(pose);
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
                AprilTagSingleDetection tag = melhorTag(tagsAlinhamento);
                double now = getRuntime();
                double dt = now - lastTime;
                lastTime = now;

                forward = 0;
                rotate = 0;

                if (now - alignStart > ALIGN_TIMEOUT_S) {
                    state = State.MANUAL;
                } else if (tag != null) {
                    double bearingErr = GOAL_X - tag.ftcPose.bearing;
                    double rangeErr = tag.ftcPose.range - DESIRED_RANGE_IN;

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
                        // range > alvo = longe demais = anda para frente
                        forward = rangeOk ? 0 : Range.clip(rangeErr * kRange, -MAX_DRIVE, MAX_DRIVE);
                    }
                } else {
                    // Tag fora de vista: gira pela odometria em direção ao hive, sem andar para frente
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
        telemetry.addData("Face usada", faceEscolhida);
        telemetry.addData("Tags procuradas", Arrays.toString(tagsAlinhamento));
        if (arcTarget != null) {
            telemetry.addData("Alvo no arco", "x %.1f  y %.1f", arcTarget.x(), arcTarget.y());
        }
        AprilTagSingleDetection t = melhorTag(tagsAlinhamento);
        if (t != null) {
            telemetry.addData("Distância (lente→tag)", "%.1f in (alvo %.1f)", t.ftcPose.range, DESIRED_RANGE_IN);
            telemetry.addData("Bearing", "%.1f°", t.ftcPose.bearing);
        } else {
            telemetry.addLine("Nenhuma tag do lado ativo visível");
        }
        telemetry.update();
    }

    @Override
    public void stop() {
        visaoTag.close();
    }

    /** Entre as tags visíveis que estão na lista, devolve a mais centralizada (menor |bearing|). */
    private AprilTagSingleDetection melhorTag(int[] ids) {
        AprilTagSingleDetection best = null;
        for (AprilTagDetection d : visaoTag.getDetections()) {
            if (!(d instanceof AprilTagSingleDetection)) continue;
            AprilTagSingleDetection s = (AprilTagSingleDetection) d;
            if (!contem(ids, s.id)) continue;
            if (best == null || Math.abs(s.ftcPose.bearing) < Math.abs(best.ftcPose.bearing)) {
                best = s;
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

    private void resetPose(double x, double y, double headingDeg) {
        follower.setPose(new Pose(x, y, Math.toRadians(headingDeg)));
    }

    private void startAlign() {
        // Escolhe a face pelo arco em que o robô está e trava até o fim do alinhamento
        boolean superior = ArcoHive.isSuperior(arcTarget);
        tagsAlinhamento = superior ? TAGS_FACE_SUPERIOR : TAGS_FACE_INFERIOR;
        faceEscolhida = superior ? "superior (arco de cima)" : "inferior (arco de baixo)";

        hasLastError = false;
        lastTime = getRuntime();
        alignStart = getRuntime();
    }
}