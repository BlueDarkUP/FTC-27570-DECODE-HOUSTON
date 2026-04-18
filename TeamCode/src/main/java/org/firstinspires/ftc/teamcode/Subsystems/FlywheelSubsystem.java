package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class FlywheelSubsystem {

    private DcMotorEx motorSH;
    private DcMotorEx motorHS;

    public static final double IDLE_VELOCITY_MIN = 3000.0;

    // ================= 7天前原版容差与边界参数 =================
    private final double RPM_LOWER_BOUND = 3000.0;
    private final double RPM_UPPER_BOUND = 5050.0;
    private final double MAX_TOLERANCE = 1000;
    private final double MIN_TOLERANCE = 45; // 恢复旧版的严格要求，解决抛物线飘的问题
    private final double SPOOL_UP_TOLERANCE = 100.0;

    // ================= 7天前原版 PIDF 参数 =================
    public static double kP = 0.011;
    public static double kI = 0.0004;
    public static double kD = 0.00000023;
    public static double kF = 0.00033;

    private final double TICKS_PER_REV = 28.0;

    private ElapsedTime timer = new ElapsedTime();
    private double lastErrorTPS = 0;
    private double integralSum = 0;

    private boolean isFlywheelReady = false;
    private double currentRPM = 0.0;
    private String activeBoost = "None";

    public FlywheelSubsystem(HardwareMap hardwareMap) {
        motorSH = hardwareMap.get(DcMotorEx.class, "SH");
        motorHS = hardwareMap.get(DcMotorEx.class, "HS");

        motorSH.setDirection(DcMotorSimple.Direction.FORWARD);
        motorHS.setDirection(DcMotorSimple.Direction.REVERSE);

        motorSH.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorHS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // 维持 FLOAT 模式，配合下文 power 最小值为 0.0 的限制
        motorSH.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorHS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public void start() {
        timer.reset();
        lastErrorTPS = 0;
        integralSum = 0;
    }

    public void update(double targetVelocityRPM, boolean isEmergencyBrake, boolean isActiveSpooling) {
        // [1] 动态容差计算
        double dynamicTolerance;
        if (targetVelocityRPM <= RPM_LOWER_BOUND) {
            dynamicTolerance = MAX_TOLERANCE;
        } else if (targetVelocityRPM >= RPM_UPPER_BOUND) {
            dynamicTolerance = MIN_TOLERANCE;
        } else {
            double ratio = (targetVelocityRPM - RPM_LOWER_BOUND) / (RPM_UPPER_BOUND - RPM_LOWER_BOUND);
            dynamicTolerance = MAX_TOLERANCE - ratio * (MAX_TOLERANCE - MIN_TOLERANCE);
        }

        double currentVelTPS = motorSH.getVelocity();
        double targetVelTPS = (targetVelocityRPM * TICKS_PER_REV) / 60.0;
        currentRPM = (currentVelTPS * 60.0) / TICKS_PER_REV;
        double errorRPM = targetVelocityRPM - currentRPM;

        // [2] 飞轮就绪状态机（保持了结构，但容差评估基于原版参数）
        if (!isActiveSpooling) {
            isFlywheelReady = false;
        } else {
            if (!isFlywheelReady) {
                if (currentRPM >= targetVelocityRPM - SPOOL_UP_TOLERANCE) {
                    isFlywheelReady = true;
                }
            } else {
                if (currentRPM < targetVelocityRPM - dynamicTolerance) {
                    isFlywheelReady = false;
                }
            }
        }

        // [3] 时间差计算
        double dt = timer.seconds();
        timer.reset();
        if (dt == 0) dt = 1e-9;

        double errorTPS = targetVelTPS - currentVelTPS;

        // [4] 积分计算与抗积分饱和 (Anti-Windup)
        if (targetVelocityRPM > 100) {
            integralSum += errorTPS * dt;
        } else {
            integralSum = 0;
        }

        double maxIntegral = 0.25;
        if (kI != 0) {
            if (integralSum > maxIntegral / kI) integralSum = maxIntegral / kI;
            if (integralSum < -maxIntegral / kI) integralSum = -maxIntegral / kI;
        }

        // [5] 微分计算
        double derivative = (errorTPS - lastErrorTPS) / dt;
        lastErrorTPS = errorTPS;

        // [6] 原版核心：PIDF 控制律
        double power = (kF * targetVelTPS) + (kP * errorTPS) + (kI * integralSum) + (kD * derivative);

        // [7] 恢复原版 Bang-Bang 极速补电机制
        double bangBangThreshold = Math.abs(dynamicTolerance) - (Math.abs(dynamicTolerance) / 7.0);
        activeBoost = "None";

        if (isActiveSpooling && targetVelocityRPM > 100) {
            if (errorRPM > bangBangThreshold) {
                power = 1.0;
                integralSum = 0;
                activeBoost = "Bang-Bang (极速补电)";
            }
        }

        // [8] 刹车或待机拦截
        if (isEmergencyBrake || targetVelocityRPM <= 0) {
            power = 0;
            integralSum = 0;
        }

        // [9] 【关键修改】强制下限为 0.0，严禁飞轮反向通电刹车导致转速剧烈震荡
        power = Math.max(0.0, Math.min(1.0, power));

        // [10] 硬件输出
        motorSH.setPower(power);
        motorHS.setPower(power);
    }

    public boolean isReady() {
        return isFlywheelReady;
    }

    public double getCurrentRPM() {
        return currentRPM;
    }

    public String getActiveBoostState() {
        return activeBoost;
    }
}