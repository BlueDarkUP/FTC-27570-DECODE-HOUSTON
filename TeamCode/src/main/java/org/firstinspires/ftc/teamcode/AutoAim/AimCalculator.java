package org.firstinspires.ftc.teamcode.AutoAim;

import com.acmerobotics.dashboard.config.Config;

@Config // 必须添加 @Config 注解，Dashboard 才能识别到这些变量
public class AimCalculator {

    // 机械发射延迟时间
    public static double MECHANICAL_SHOOT_DELAY = 0.1;

    // =========================================================
    // 通过 FTC Dashboard 实时调参的弹道数据点 (必须是 public static)
    // 调参提示：请确保 P1 到 P7 的 DIST 是【从小到大】递增的！
    // 如果发现某个距离不准，直接修改离该距离最近的 P 点即可。
    // =========================================================

    // 数据点 1：贴脸近战
    public static double P1_DIST = 21.0;
    public static double P1_RPM = 2800;
    public static double P1_PITCH = 0.35;
    public static double P1_TIME = 0.3;

    // 数据点 2：中近场
    public static double P2_DIST = 60.0;
    public static double P2_RPM = 3200;
    public static double P2_PITCH = 0.70;
    public static double P2_TIME = 0.4;

    // 数据点 3：中场 (新增)
    public static double P3_DIST = 67.1;
    public static double P3_RPM = 3250;
    public static double P3_PITCH = 0.78;
    public static double P3_TIME = 0.5;

    // 数据点 4：中远场
    public static double P4_DIST = 89.2;
    public static double P4_RPM = 3800;
    public static double P4_PITCH = 1.0;
    public static double P4_TIME = 0.5;

    // 数据点 5：远场 1 (新增)
    public static double P5_DIST = 104.6;
    public static double P5_RPM = 3900;
    public static double P5_PITCH = 1.0;
    public static double P5_TIME = 0.8;

    // 数据点 6：远场 2 (新增)
    public static double P6_DIST = 122.3;
    public static double P6_RPM = 4300;
    public static double P6_PITCH = 1.0;
    public static double P6_TIME = 0.8;

    // 数据点 7：极限远场
    public static double P7_DIST = 150.0;
    public static double P7_RPM = 4900;
    public static double P7_PITCH = 1.0;
    public static double P7_TIME = 0.9;

    /**
     * 每次解算时动态获取最新的 Dashboard 参数
     * 增加数据点可以大幅提升线性插值的鲁棒性和顺滑度
     */
    private static double[][] getShootData() {
        return new double[][] {
                {P1_DIST, P1_RPM, P1_PITCH, P1_TIME},
                {P2_DIST, P2_RPM, P2_PITCH, P2_TIME},
                {P3_DIST, P3_RPM, P3_PITCH, P3_TIME},
                {P4_DIST, P4_RPM, P4_PITCH, P4_TIME},
                {P5_DIST, P5_RPM, P5_PITCH, P5_TIME},
                {P6_DIST, P6_RPM, P6_PITCH, P6_TIME},
                {P7_DIST, P7_RPM, P7_PITCH, P7_TIME}
        };
    }

    /**
     * 核心的分段线性插值算法
     * @param dist 当前目标距离
     * @param col  需要获取的数据列索引 (1:RPM, 2:Pitch, 3:Time)
     */
    public static double interpolate(double dist, int col) {
        double[][] currentData = getShootData();

        // 边界保护：如果距离比最近的点还近，直接用最近点的值
        if (dist <= currentData[0][0]) return currentData[0][col];

        // 边界保护：如果距离比最远的点还远，直接用最远点的值
        int lastIdx = currentData.length - 1;
        if (dist >= currentData[lastIdx][0]) return currentData[lastIdx][col];

        // 遍历寻找当前距离所在的区间，进行一次线性插值
        for (int i = 0; i < lastIdx; i++) {
            if (dist >= currentData[i][0] && dist <= currentData[i + 1][0]) {
                double d1 = currentData[i][0],   d2 = currentData[i + 1][0];
                double v1 = currentData[i][col], v2 = currentData[i + 1][col];

                // 线性插值公式: y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
                return v1 + (dist - d1) * (v2 - v1) / (d2 - d1);
            }
        }
        return currentData[0][col]; // 理论上不会走到这里，作为保底
    }

    // =========================================================
    // 运动学解算核心逻辑 (包含底盘运动补偿)
    // =========================================================

    public static AimResult solveAim(double rx, double ry, double vx, double vy, double ax, double ay, double tx, double ty) {
        // 计算当前底盘到目标的理论距离
        double dx = tx - rx;
        double dy = ty - ry;
        double dist = Math.hypot(dx, dy);

        // 防呆：距离过近不解算
        if (dist < 0.1) return null;

        // 获取初始距离下的预估飞行时间
        double tf = interpolate(dist, 3);
        double vDist = dist;

        // 迭代两次：补偿底盘在开枪延迟+子弹飞行期间的位移
        for (int i = 0; i < 2; i++) {
            double totalTime = tf + MECHANICAL_SHOOT_DELAY;

            // 预测底盘位移 (s = vt + 0.5at^2)
            double chassisDisplacementX = (vx * totalTime) + (0.5 * ax * totalTime * totalTime);
            double chassisDisplacementY = (vy * totalTime) + (0.5 * ay * totalTime * totalTime);

            // 虚拟目标点
            double predictedX = tx - rx - chassisDisplacementX;
            double predictedY = ty - ry - chassisDisplacementY;

            // 重新计算等效距离并更新飞行时间
            vDist = Math.hypot(predictedX, predictedY);
            tf = interpolate(vDist, 3);
        }

        // 最终解算所需的参数
        double finalTotalTime = tf + MECHANICAL_SHOOT_DELAY;
        double finalChassisDispX = (vx * finalTotalTime) + (0.5 * ax * finalTotalTime * finalTotalTime);
        double finalChassisDispY = (vy * finalTotalTime) + (0.5 * ay * finalTotalTime * finalTotalTime);

        double finalDX = dx - finalChassisDispX;
        double finalDY = dy - finalChassisDispY;

        // 构建并返回最终结果
        return new AimResult(
                vDist,                                          // 等效距离
                Math.toDegrees(Math.atan2(finalDY, finalDX)),   // 补偿后的 Yaw 角度
                interpolate(vDist, 1),                          // 插值计算 RPM
                interpolate(vDist, 2),                          // 插值计算 Pitch
                tf                                              // 预测的子弹飞行时间
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