package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

import static com.pedropathing.api.Paths.line;

/**
 * Geometria dos dois arcos de tiro em volta do hive (Pedro 3).
 * Arco inferior: definido por ARC_START_DEG..ARC_END_DEG (ângulos a partir do centro do hive).
 * Arco superior: espelhado pelo eixo horizontal do hive (ângulo θ vira -θ).
 * Coordenadas: polegadas, 0..144, origem no canto inferior esquerdo, heading em radianos.
 */
@Config
public class ArcoHive {

    // Hive: ASSUMIDO centralizado em X e a 128 cm da parede de baixo (y = 0). Confirme.
    public static double HIVE_X = 72;
    public static double HIVE_WALL_DIST_CM = 128;

    // Distância do SHOOTER até o hive. O arco é feito pelo centro do robô.
    public static double SHOOTER_DIST_CM = 115;
    public static double SHOOTER_OFFSET_CM = 0;

    // Arco inferior (0° = +X, anti-horário)
    public static double ARC_START_DEG = 216, ARC_END_DEG = 324;

    public static double HEADING_OFFSET_DEG = 0;   // ajuste de montagem do heading
    public static double ON_ARC_TOLERANCE = 1.0;   // polegadas

    private static final double CM_TO_IN = 1.0 / 2.54;

    public static double hiveY() {
        return HIVE_WALL_DIST_CM * CM_TO_IN;
    }

    public static double arcRadius() {
        return (SHOOTER_DIST_CM - SHOOTER_OFFSET_CM) * CM_TO_IN;
    }

    /** Heading que aponta do ponto (x, y) para o centro do hive. */
    public static double headingToHive(double x, double y) {
        return Math.atan2(hiveY() - y, HIVE_X - x);
    }

    /** Ponto mais perto de 'robot' em um arco definido por ângulos (graus, anti-horário). */
    private static Pose nearestPoseOnArc(Pose robot, double startDeg, double endDeg) {
        double dx = robot.x() - HIVE_X;
        double dy = robot.y() - hiveY();
        double theta = normalize360(Math.toDegrees(Math.atan2(dy, dx)));

        double start = normalize360(startDeg);
        double span = normalize360(endDeg - startDeg);
        double fromStart = normalize360(theta - start);

        double phi;
        if (fromStart <= span) {
            phi = theta;
        } else {
            double toStart = Math.min(fromStart, 360 - fromStart);
            double toEnd = Math.abs(normalize180(theta - endDeg));
            phi = (toStart < toEnd) ? start : normalize360(endDeg);
        }

        double px = HIVE_X + arcRadius() * Math.cos(Math.toRadians(phi));
        double py = hiveY() + arcRadius() * Math.sin(Math.toRadians(phi));
        return new Pose(px, py, headingToHive(px, py) + Math.toRadians(HEADING_OFFSET_DEG));
    }

    /** Ponto do arco de tiro MAIS PRÓXIMO (inferior ou superior), já com heading para o hive. */
    public static Pose nearestArcPose(Pose robot) {
        Pose inferior = nearestPoseOnArc(robot, ARC_START_DEG, ARC_END_DEG);
        // Espelho pelo eixo horizontal do hive: θ -> -θ, então 216..324 vira 36..144
        Pose superior = nearestPoseOnArc(robot, 360 - ARC_END_DEG, 360 - ARC_START_DEG);
        return distance(robot, inferior) <= distance(robot, superior) ? inferior : superior;
    }

    /** True se o ponto está no arco superior (acima do centro do hive). */
    public static boolean isSuperior(Pose arcPose) {
        return arcPose.y() > hiveY();
    }

    /** Caminho em linha reta até 'to', com o heading final pelo caminho mais curto. */
    public static Path buildPath(Pose from, Pose to) {
        double endHeading = from.heading()
                + Math.toRadians(normalize180(Math.toDegrees(to.heading() - from.heading())));
        Pose end = new Pose(to.x(), to.y(), endHeading);
        return line(from, end).linear(from, end);
    }

    public static double distance(Pose a, Pose b) {
        return Math.hypot(a.x() - b.x(), a.y() - b.y());
    }

    public static double normalize360(double a) {
        a %= 360;
        return a < 0 ? a + 360 : a;
    }

    public static double normalize180(double a) {
        a = normalize360(a);
        return a > 180 ? a - 360 : a;
    }
}