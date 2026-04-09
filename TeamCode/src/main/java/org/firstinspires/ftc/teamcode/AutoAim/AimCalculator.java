package org.firstinspires.ftc.teamcode.AutoAim;

import com.acmerobotics.dashboard.config.Config;

@Config // 必须添加 @Config 注解，Dashboard 才能识别到这些变量
public class AimCalculator {

    // =========================================================
    // 通过 FTC Dashboard 实时调参的弹道数据点 (必须是 public static)
    // 格式: 距离(inch), RPM, Pitch, 飞行时间(s)
    // =========================================================

    // 数据点 1：近战
    public static double P1_DIST = 21.0;
    public static double P1_RPM = 1550.0;
    public static double P1_PITCH = 0.4;
    public static double P1_TIME = 0.15;

    // 数据点 2：中近
    public static double P2_DIST = 60.0;
    public static double P2_RPM = 1600.0;
    public static double P2_PITCH = 0.5;
    public static double P2_TIME = 0.35;

    // 数据点 3：中远
    public static double P3_DIST = 89.2;
    public static double P3_RPM = 1800.0;
    public static double P3_PITCH = 0.75;
    public static double P3_TIME = 0.55;

    // 数据点 4：最远
    public static double P4_DIST = 150.0;
    public static double P4_RPM = 2500.0;
    public static double P4_PITCH = 1;
    public static double P4_TIME = 0.90;

    /**
     * 每次解算时动态获取最新的 Dashboard 参数
     */
    private static double[][] getShootData() {
        return new double[][] {
                {P1_DIST, P1_RPM, P1_PITCH, P1_TIME},
                {P2_DIST, P2_RPM, P2_PITCH, P2_TIME},
                {P3_DIST, P3_RPM, P3_PITCH, P3_TIME},
                {P4_DIST, P4_RPM, P4_PITCH, P4_TIME}
        };
    }

    public static double interpolate(double dist, int col) {
        double[][] currentData = getShootData(); // 获取实时更新的数组

        if (dist <= currentData[0][0]) return currentData[0][col];
        if (dist >= currentData[currentData.length - 1][0]) return currentData[currentData.length - 1][col];

        for (int i = 0; i < currentData.length - 1; i++) {
            if (dist >= currentData[i][0] && dist <= currentData[i + 1][0]) {
                double d1 = currentData[i][0], d2 = currentData[i + 1][0];
                double v1 = currentData[i][col], v2 = currentData[i + 1][col];
                return v1 + (dist - d1) * (v2 - v1) / (d2 - d1);
            }
        }
        return currentData[0][col];
    }

    public static AimResult solveAim(double rx, double ry, double vx, double vy, double tx, double ty) {
        double dx = tx - rx;
        double dy = ty - ry;
        double dist = Math.hypot(dx, dy);
        if (dist < 0.1) return null;

        double tf = interpolate(dist, 3);
        double vDist = dist;

        // 迭代两次以提高预测精度
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