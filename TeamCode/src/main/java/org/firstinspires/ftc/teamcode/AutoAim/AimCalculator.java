package org.firstinspires.ftc.teamcode.AutoAim;

class AimCalculator {
    private static final double[][] SHOOT_DATA = {
            {20.0, 2100.0, 0.35, 0.15},
            {60.0, 2720.0, 0.42, 0.35},
            {90.0, 3050.0, 0.50, 0.55},
            {150.0, 3400.0, 0.65, 0.90}
    };

    public static double interpolate(double dist, int col) {
        if (dist <= SHOOT_DATA[0][0]) return SHOOT_DATA[0][col];
        if (dist >= SHOOT_DATA[SHOOT_DATA.length - 1][0]) return SHOOT_DATA[SHOOT_DATA.length - 1][col];
        for (int i = 0; i < SHOOT_DATA.length - 1; i++) {
            if (dist >= SHOOT_DATA[i][0] && dist <= SHOOT_DATA[i + 1][0]) {
                double d1 = SHOOT_DATA[i][0], d2 = SHOOT_DATA[i + 1][0];
                double v1 = SHOOT_DATA[i][col], v2 = SHOOT_DATA[i + 1][col];
                return v1 + (dist - d1) * (v2 - v1) / (d2 - d1);
            }
        }
        return SHOOT_DATA[0][col];
    }

    public static AimResult solveAim(double rx, double ry, double vx, double vy, double tx, double ty) {
        double dx = tx - rx;
        double dy = ty - ry;
        double dist = Math.hypot(dx, dy);
        if (dist < 0.1) return null;

        double tf = interpolate(dist, 3);
        double vDist = dist;

        for (int i = 0; i < 2; i++) {
            double predictedX = tx - (vx * tf);
            double predictedY = ty - (vy * tf);
            vDist = Math.hypot(predictedX - rx, predictedY - ry);
            tf = interpolate(vDist, 3);
        }

        double finalDX = dx - vx * tf;
        double finalDY = dy - vy * tf;

        return new AimResult(
                vDist,
                Math.toDegrees(Math.atan2(finalDY, finalDX)),
                interpolate(vDist, 1),
                interpolate(vDist, 2),
                tf
        );
    }

    public static class AimResult {
        public double dist, algYaw, rpm, pitch, flightTime;
        public AimResult(double d, double y, double r, double p, double t) {
            this.dist = d; this.algYaw = y; this.rpm = r; this.pitch = p; this.flightTime = t;
        }
    }
}