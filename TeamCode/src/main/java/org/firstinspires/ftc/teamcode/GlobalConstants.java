package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class GlobalConstants {

    // =========================================
    // 1. 硬件核心参数 (Ticks / Offsets)
    // =========================================
    public static double TURRET_TICKS_PER_REV = 31087.589;
    public static double FLYWHEEL_TICKS_PER_REV = 28.0;

    // =========================================
    // 2. 俯仰机构 (Pitch) - 已同步为最新数据
    // =========================================
    public static double PITCH_LP_UP = 0.9;
    public static double PITCH_LP_DOWN = 0.2;
    public static double PITCH_RP_UP = 0.0;
    public static double PITCH_RP_DOWN = 0.7;

    // 自动阶段常用的固定俯仰档位 (消除代码里的魔法数字)
    public static double PITCH_POS_DEFAULT = 1.0;         // 默认高位 (常用1.0)
    public static double PITCH_POS_TRANSIT = 0.87;        // 寻路/过渡姿态
    public static double PITCH_POS_INTAKE_NORMAL = 0.75;  // 常规吸取姿态
    public static double PITCH_POS_INTAKE_DEEP = 0.70;    // 开门/深吸姿态

    // =========================================
    // 3. 拨弹舵机 (BBB)
    // =========================================
    public static double BBB_IDLE_POS = 0.0;
    public static double BBB_SHOOT_POS = 0.18;
    public static double BBB_DELAY_MS = 300.0;

    // =========================================
    // 4. 飞轮转速 (Flywheel RPM)
    // =========================================
    public static double FLYWHEEL_RPM_MIN = 3000.0;
    public static double FLYWHEEL_RPM_MAX = 5050.0;

    // 自动阶段常用的固定转速
    public static double AUTO_RPM_NORMAL = 3250.0;        // 绝大部分的近点发射
    public static double AUTO_RPM_DOOR_1 = 3350.0;        // 开门后第一排略高的转速
    public static double AUTO_RPM_DOOR_2 = 3400.0;        // 极速开门吸取转速
    public static double AUTO_RPM_FAR = 4700.0;           // 蓝方远点第三排极限转速

    // =========================================
    // 5. 射击时间常数 (Shooter Timing)
    // =========================================
    public static double SHOOT_TIME_LONG = 0.8;           // 远点长时射击
    public static double SHOOT_TIME_NORMAL = 0.5;         // 标准射击时间
    public static double SHOOT_TIME_SHORT = 0.45;         // 极限提速射击时间
}