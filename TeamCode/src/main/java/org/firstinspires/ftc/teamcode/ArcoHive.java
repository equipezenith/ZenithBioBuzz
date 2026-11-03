package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.line;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

@Config
public class ArcoHive {

    public static double HIVE_X = 72;
    public static double HIVE_WALL_DIST_CM = 128;

    public static double SHOOTER_DIST_CM = 115;
    public static double SHOOTER_OFFSET_CM = 0;

    public static double ARC_START_DEG = 210, ARC_END_DEG = 330;

    public static double HEADING_OFFSET_DEG = 0;
    public static double ON_ARC_TOLERANCE = 1.0;

    private static final double CM_TO_IN = 1.0 / 2.54;

    public static double hiveY() {
        return HIVE_WALL_DIST_CM * CM_TO_IN;
    }

    public static double arcRadius() {
        return (SHOOTER_DIST_CM - SHOOTER_OFFSET_CM) * CM_TO_IN;
    }

    public static double headingToHive(double x, double y) {
        return Math.atan2(hiveY() - y, HIVE_X - x);
    }

    public static Pose nearestArcPose(Pose robot) {
        double dx = robot.x() - HIVE_X;
        double dy = robot.y() - hiveY();
        double theta = normalize360(Math.toDegrees(Math.atan2(dy, dx)));

        double start = normalize360(ARC_START_DEG);
        double span = normalize360(ARC_END_DEG - ARC_START_DEG);
        double fromStart = normalize360(theta - start);

        double phi;
        if (fromStart <= span) {
            phi = theta;
        } else {
            double toStart = Math.min(fromStart, 360 - fromStart);
            double toEnd = Math.abs(normalize180(theta - ARC_END_DEG));
            phi = (toStart < toEnd) ? start : normalize360(ARC_END_DEG);
        }

        double px = HIVE_X + arcRadius() * Math.cos(Math.toRadians(phi));
        double py = hiveY() + arcRadius() * Math.sin(Math.toRadians(phi));
        return new Pose(px, py, headingToHive(px, py) + Math.toRadians(HEADING_OFFSET_DEG));
    }

    /** Linha reta até 'to', com o heading final pelo caminho mais curto. */
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