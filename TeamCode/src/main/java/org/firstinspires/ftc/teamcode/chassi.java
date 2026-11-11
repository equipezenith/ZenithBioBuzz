package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.vision.VisionPortal;

public class chassi {

    /** Manche abaixo disso não conta como "o motorista mexeu". */
    public static double ZONA_MORTA = 0.05;

    private final Follower follower;
    private boolean botaoAnterior = false;

    public Sistema_visao vision_tag = new Sistema_visao();
    int[] viewIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.VERTICAL);

    public chassi(HardwareMap hardwareMap, Pose poseInicial) {
        //vision_tag.init(hardwareMap, "CAMERA_TAG", Sistema_visao.Mode.APRILTAG, viewIds[1]);

        follower = Constants.create(hardwareMap);
        follower.setPose(poseInicial);
        follower.update();
    }

    /** Chame UMA vez por ciclo do loop, antes das outras funções. */
    public void atualizar() {
        follower.update();
    }

    /** Anda (robot-centric) com prioridade sobre o alinhamento. Sem manche, segura a posição. */
    public void andar(double frente, double lateral, double giro) {
        boolean mexeu = Math.abs(frente) > ZONA_MORTA
                || Math.abs(lateral) > ZONA_MORTA
                || Math.abs(giro) > ZONA_MORTA;

        if (mexeu) {
            ManualDrive.driveOrHold(follower, frente, lateral, giro);
        } else if (!alinhando()) {
            ManualDrive.driveOrHold(follower, 0, 0, 0);   // hold
        }
    }

    public void andar(Gamepad gamepad) {
        andar(-gamepad.left_stick_y, gamepad.left_stick_x, gamepad.right_stick_x);
    }

    /** Igual ao andar(gamepad), com escala (0.3 = 30%, para o modo lento). */
    public void andar(Gamepad gamepad, double escala) {
        andar(-gamepad.left_stick_y * escala,
                gamepad.left_stick_x * escala,
                gamepad.right_stick_x * escala);
    }

    /** Vai até o ponto do arco mais perto. Retorna true se começou a andar. */
    public boolean alinharHive() {
        Pose atual = follower.pose();
        Pose alvo = ArcoHive.nearestArcPose(atual);
        if (ArcoHive.distance(atual, alvo) <= ArcoHive.ON_ARC_TOLERANCE) return false;

        follower.follow(ArcoHive.buildPath(atual, alvo));
        return true;
    }

    /** Versão para botão: só dispara quando o botão é apertado. */
    public boolean alinharHive(boolean botao) {
        boolean disparou = botao && !botaoAnterior;
        botaoAnterior = botao;
        return disparou && alinharHive();
    }

    public void cancelarAlinhamento() {
        ManualDrive.driveOrHold(follower, 0, 0, 0);
    }

    /** true enquanto o robô ainda está indo ao arco. */
    public boolean alinhando() {
        return "FOLLOW".equals(String.valueOf(follower.mode())) && follower.isBusy();
    }

    public Pose pose() { return follower.pose(); }
    public void setPose(Pose pose) { follower.setPose(pose); }
    public Follower follower() { return follower; }
}