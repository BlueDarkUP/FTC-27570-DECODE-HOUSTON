package org.firstinspires.ftc.teamcode.AutoAim;

import com.acmerobotics.dashboard.config.Config;

@Config
public class AimCalculator {
    public static double MECHANICAL_SHOOT_DELAY = 0;
    public static double CONSTANT_FLIGHT_TIME = 0.7;

    public static double P1_DIST = 21.0;
    public static double P1_RPM = 2800;
    public static double P1_PITCH = 0;

    public static double P2_DIST = 60.0;
    public static double P2_RPM = 3000;
    public static double P2_PITCH = 0.65;

    public static double P3_DIST = 67.1;
    public static double P3_RPM = 3200;
    public static double P3_PITCH = 0.78;

    public static double P4_DIST = 89.2;
    public static double P4_RPM = 3600;
    public static double P4_PITCH = 1.0;

    public static double P5_DIST = 104.6;
    public static double P5_RPM = 3800;
    public static double P5_PITCH = 1.0;

    public static double P6_DIST = 122.3;
    public static double P6_RPM = 4000;
    public static double P6_PITCH = 1.0;

    // --- 远射区数据 (Flatline 理念，消除里程计抖动带来的 RPM 震荡) ---
    public static double P7_DIST = 140.0;
    public static double P7_RPM = 3920;
    public static double P7_PITCH = 1.0;

    public static double P8_DIST = 160.0;
    public static double P8_RPM = 3920;
    public static double P8_PITCH = 1.0;

    // --- 远射电压补偿参数 ---
    public static double VOLTAGE_BASE = 12.3;         // 5150转速时的基准电压
    public static double RPM_PER_VOLT_FAR = -300.0;   // 每高于基准电压 1V，远射RPM下降的数值

    private static double[][] getShootData() {
        return new double[][] {
                {P1_DIST, P1_RPM, P1_PITCH},
                {P2_DIST, P2_RPM, P2_PITCH},
                {P3_DIST, P3_RPM, P3_PITCH},
                {P4_DIST, P4_RPM, P4_PITCH},
                {P5_DIST, P5_RPM, P5_PITCH},
                {P6_DIST, P6_RPM, P6_PITCH},
                {P7_DIST, P7_RPM, P7_PITCH},
                {P8_DIST, P8_RPM, P8_PITCH}
        };
    }

    public static double interpolate(double dist, int col, double currentVoltage) {
        double[][] currentData = getShootData();
        double result = currentData[0][col];

        // 基础多项式插值逻辑
        if (dist <= currentData[0][0]) {
            result = currentData[0][col];
        } else {
            int lastIdx = currentData.length - 1;
            if (dist >= currentData[lastIdx][0]) {
                result = currentData[lastIdx][col];
            } else {
                for (int i = 0; i < lastIdx; i++) {
                    if (dist >= currentData[i][0] && dist <= currentData[i + 1][0]) {
                        double d1 = currentData[i][0],   d2 = currentData[i + 1][0];
                        double v1 = currentData[i][col], v2 = currentData[i + 1][col];

                        result = v1 + (dist - d1) * (v2 - v1) / (d2 - d1);
                        break;
                    }
                }
            }
        }

        if (col == 1 && dist >= P7_DIST) {
            double voltageDiff = currentVoltage - VOLTAGE_BASE;
            double rpmCompensation = voltageDiff * RPM_PER_VOLT_FAR;
            result += rpmCompensation;
        }

        return result;
    }

    public static AimResult solveAim(double futureRx, double futureRy, double tx, double ty, double currentVoltage) {
        double dx = tx - futureRx;
        double dy = ty - futureRy;
        double vDist = Math.hypot(dx, dy);

        if (vDist < 0.1) return null;

        return new AimResult(
                vDist,
                Math.toDegrees(Math.atan2(dy, dx)),
                interpolate(vDist, 1, currentVoltage),
                interpolate(vDist, 2, currentVoltage),
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