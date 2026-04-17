package org.firstinspires.ftc.teamcode.AutoAim;

import com.acmerobotics.dashboard.config.Config;

@Config
public class AimCalculator {
    public static double MECHANICAL_SHOOT_DELAY = 0.1;
    public static double CONSTANT_FLIGHT_TIME = 0.85;

    public static double P1_DIST = 21.0;
    public static double P1_RPM = 2800;
    public static double P1_PITCH = 0.35;

    public static double P2_DIST = 60.0;
    public static double P2_RPM = 3150;
    public static double P2_PITCH = 0.70;

    public static double P3_DIST = 67.1;
    public static double P3_RPM = 3200;
    public static double P3_PITCH = 0.78;

    public static double P4_DIST = 89.2;
    public static double P4_RPM = 3750;
    public static double P4_PITCH = 1.0;

    public static double P5_DIST = 104.6;
    public static double P5_RPM = 3750;
    public static double P5_PITCH = 1.0;

    public static double P6_DIST = 122.3;
    public static double P6_RPM = 4250;
    public static double P6_PITCH = 1.0;

    public static double P7_DIST = 150.0;
    public static double P7_RPM = 4900;
    public static double P7_PITCH = 1.0;

    private static double[][] getShootData() {
        return new double[][] {
                {P1_DIST, P1_RPM, P1_PITCH},
                {P2_DIST, P2_RPM, P2_PITCH},
                {P3_DIST, P3_RPM, P3_PITCH},
                {P4_DIST, P4_RPM, P4_PITCH},
                {P5_DIST, P5_RPM, P5_PITCH},
                {P6_DIST, P6_RPM, P6_PITCH},
                {P7_DIST, P7_RPM, P7_PITCH}
        };
    }

    public static double interpolate(double dist, int col) {
        double[][] currentData = getShootData();

        if (dist <= currentData[0][0]) return currentData[0][col];

        int lastIdx = currentData.length - 1;
        if (dist >= currentData[lastIdx][0]) return currentData[lastIdx][col];

        for (int i = 0; i < lastIdx; i++) {
            if (dist >= currentData[i][0] && dist <= currentData[i + 1][0]) {
                double d1 = currentData[i][0],   d2 = currentData[i + 1][0];
                double v1 = currentData[i][col], v2 = currentData[i + 1][col];

                return v1 + (dist - d1) * (v2 - v1) / (d2 - d1);
            }
        }
        return currentData[0][col];
    }

    public static AimResult solveAim(double futureRx, double futureRy, double tx, double ty) {
        double dx = tx - futureRx;
        double dy = ty - futureRy;
        double vDist = Math.hypot(dx, dy);

        if (vDist < 0.1) return null;

        return new AimResult(
                vDist,
                Math.toDegrees(Math.atan2(dy, dx)),
                interpolate(vDist, 1),
                interpolate(vDist, 2),
                CONSTANT_FLIGHT_TIME
        );
    }

    public static class AimResult {
        public double dist, algYaw, rpm, pitch, flightTime;
        public AimResult(double d, double y, double r, double p, double t) {
            this.dist = d;
            this.algYaw = y;
            this.rpm = r;
            this.pitch = p;
            this.flightTime = t;
        }
    }
}