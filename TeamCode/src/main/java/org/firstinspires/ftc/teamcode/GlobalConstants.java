package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

@Config
public class GlobalConstants {

    // =========================================
    // 1. 硬件核心参数 (Ticks / Offsets)
    // =========================================
    public static double TURRET_TICKS_PER_REV = 32768;
    public static double FLYWHEEL_TICKS_PER_REV = 28.0;

    // =========================================
    // 2. 飞轮 (Flywheel) PID 与 容差参数
    // =========================================
    public static double FLYWHEEL_kP = 0.011;
    public static double FLYWHEEL_kI = 0.0004;
    public static double FLYWHEEL_kD = 0.00000023;
    public static double FLYWHEEL_kF = 0.00033;

    public static double FLYWHEEL_MAX_TOLERANCE = 1000.0;     // 动态容差上限
    public static double FLYWHEEL_MIN_TOLERANCE = 1000.0;     // 动态容差下限
    public static double FLYWHEEL_SPOOL_UP_TOLERANCE = 100.0; // 加速就绪容差

    public static double FLYWHEEL_RPM_MIN = 3000.0;
    public static double FLYWHEEL_RPM_MAX = 5050.0;

    // =========================================
    // 3. 俯仰机构 (Pitch)
    // =========================================
    public static double PITCH_LP_UP = 1;
    public static double PITCH_LP_DOWN = 0.2;
    public static double PITCH_RP_UP = 0.1;
    public static double PITCH_RP_DOWN = 0.7;

    public static double PITCH_POS_DEFAULT = 1.0;
    public static double PITCH_POS_TRANSIT = 0.77;
    public static double PITCH_POS_INTAKE_NORMAL = 0.75;
    public static double PITCH_POS_INTAKE_DEEP = 0.6;

    // =========================================
    // 4. 拨弹舵机 (BBB)
    // =========================================
    public static double BBB_IDLE_POS = 0.0;
    public static double BBB_SHOOT_POS = 0.18;
    public static double BBB_DELAY_MS = 300.0;

    // =========================================
    // 5. 自动阶段常用参数
    // =========================================
    public static double AUTO_RPM_NORMAL = 3250.0;
    public static double AUTO_RPM_DOOR_1 = 3350.0;
    public static double AUTO_RPM_DOOR_2 = 3400.0;
    public static double AUTO_RPM_FAR = 4700.0;

    public static double SHOOT_TIME_LONG = 0.8;
    public static double SHOOT_TIME_NORMAL = 0.55;
    public static double SHOOT_TIME_SHORT = 0.45;
}