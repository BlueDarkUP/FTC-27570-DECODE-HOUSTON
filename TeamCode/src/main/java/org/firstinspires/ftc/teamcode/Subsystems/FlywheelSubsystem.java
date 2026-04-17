package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class FlywheelSubsystem {

    private DcMotorEx motorSH;
    private DcMotorEx motorHS;
    private VoltageSensor batteryVoltageSensor;
    private HardwareMap hardwareMapRef;

    // 常量定义
    public static final double IDLE_VELOCITY_MIN = 3000.0;
    public static final double PRE_SPOOL_MAX = 4100.0;
    public static final double SHOOT_MAX = 4900.0;

    private final double RPM_LOWER_BOUND = 4250.0;
    private final double RPM_UPPER_BOUND = 4850.0;
    private final double MAX_TOLERANCE = 1000;
    private final double MIN_TOLERANCE = 50;
    private final double SPOOL_UP_TOLERANCE = 50.0;

    public static double kP = 0.025;
    public static double kI = 0.000;
    public static double kD = 0.000001;
    public static double kV = 0.0002;
    public static double kS = 0.03;
    public static double kA = 0.0002;

    // 新增：高阶控制常数
    public static double D_FILTER_ALPHA = 0.9; // 微分滤波器系数
    public static final double RECOVERY_DROP_THRESHOLD = 150.0; // 掉速恢复阈值 (RPM)

    private final double TICKS_PER_REV = 28.0;

    private ElapsedTime timer = new ElapsedTime();
    private ElapsedTime voltageTimer = new ElapsedTime(); // 新增：用于电压降频读取

    private double lastErrorTPS = 0;
    private double integralSum = 0;
    private double filteredDerivative = 0; // 新增：滤波后的D项
    private double lastTargetVelTPS = 0;   // 新增：用于计算目标加速度

    private boolean isFlywheelReady = false;
    private double currentRPM = 0.0;
    private String activeBoost = "None (纯PIDF)";
    private double cachedVoltage = 12.9;   // 新增：电压缓存

    public FlywheelSubsystem(HardwareMap hardwareMap) {
        this.hardwareMapRef = hardwareMap;

        motorSH = hardwareMap.get(DcMotorEx.class, "SH");
        motorHS = hardwareMap.get(DcMotorEx.class, "HS");

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        cachedVoltage = getBatteryVoltage();

        motorSH.setDirection(DcMotorSimple.Direction.FORWARD);
        motorHS.setDirection(DcMotorSimple.Direction.REVERSE);

        motorSH.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorHS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // BRAKE 模式，在 power 为 0 时提供更强的阻尼，配合反转电流更快减速
        motorSH.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorHS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /** 健壮的电池电压读取，防止断连报错 */
    private double getBatteryVoltage() {
        double maxVoltage = 0;
        for (VoltageSensor sensor : hardwareMapRef.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > maxVoltage) {
                maxVoltage = v;
            }
        }
        return Math.max(8.0, maxVoltage > 0 ? maxVoltage : 12.9);
    }

    public void start() {
        timer.reset();
        voltageTimer.reset();
        lastErrorTPS = 0;
        integralSum = 0;
        lastTargetVelTPS = 0;
    }

    public void update(double targetVelocityRPM, boolean isEmergencyBrake, boolean isActiveSpooling) {
        // 1. 降频读取电压 (每250ms读取一次，极大优化主循环速度)
        if (voltageTimer.milliseconds() > 250) {
            cachedVoltage = getBatteryVoltage();
            voltageTimer.reset();
        }

        // 动态容差计算
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

        // 就绪状态逻辑
        if (!isActiveSpooling) {
            isFlywheelReady = false;
        } else {
            if (!isFlywheelReady) {
                if (currentRPM >= targetVelocityRPM - SPOOL_UP_TOLERANCE) isFlywheelReady = true;
            } else {
                if (currentRPM < targetVelocityRPM - dynamicTolerance) isFlywheelReady = false;
            }
        }

        double dt = timer.seconds();
        if (dt <= 0.0001) dt = 0.0001; // 防止除以极小的数
        timer.reset();

        // 计算目标加速度 (用于 kA)
        double targetAccelTPS = (targetVelTPS - lastTargetVelTPS) / dt;
        lastTargetVelTPS = targetVelTPS;

        double errorTPS = targetVelTPS - currentVelTPS;

        // 2. 掉速恢复逻辑 (Bang-Bang Boost)
        double rpmError = targetVelocityRPM - currentRPM;
        boolean inRecoveryMode = false;

        // 当目标转速大于待机且掉速超过阈值时，强行切入全功率恢复
        if (targetVelocityRPM > IDLE_VELOCITY_MIN && rpmError > RECOVERY_DROP_THRESHOLD) {
            inRecoveryMode = true;
            activeBoost = "Active (急加速恢复)";
        } else {
            activeBoost = "None (纯PIDF)";
        }

        // 3. 微分项加入低通滤波 (防止读数毛刺引起的高频抖动)
        double rawDerivative = (errorTPS - lastErrorTPS) / dt;
        filteredDerivative = (D_FILTER_ALPHA * rawDerivative) + ((1.0 - D_FILTER_ALPHA) * filteredDerivative);
        lastErrorTPS = errorTPS;

        // 积分逻辑 (恢复模式下停止积分，防止 Windup)
        if (targetVelocityRPM > 100 && !inRecoveryMode) {
            integralSum += errorTPS * dt;
        } else {
            integralSum = 0;
        }

        double maxIntegral = 0.25;
        if (kI != 0) {
            if (integralSum > maxIntegral / kI) integralSum = maxIntegral / kI;
            if (integralSum < -maxIntegral / kI) integralSum = -maxIntegral / kI;
        }

        double power = 0.0;

        // 4. 停机与刹车优先
        if (targetVelocityRPM <= 0 || isEmergencyBrake) {
            power = 0;
            integralSum = 0;
        }
        // 5. 如果处于射击掉速状态，无视 PID 直接给满功率，实现秒级拉回
        else if (inRecoveryMode) {
            power = 1.0;
        }
        // 6. 常规高精度 FF + PIDF 计算
        else {
            // 前馈：克服静摩擦 + 维持当前速度 + 满足加速度需求
            double feedforward = kS * Math.signum(targetVelTPS)
                    + kV * targetVelTPS
                    + kA * targetAccelTPS;

            double pid = (kP * errorTPS) + (kI * integralSum) + (kD * filteredDerivative);

            // 基础功率计算与动态电压补偿 (基于 12.9V)
            double basePower = feedforward + pid;
            power = basePower * (12.38 / cachedVoltage);
        }

        // 限制在 -1.0 到 1.0 之间，允许负数产生反转电流快速减速
        power = Math.max(-1.0, Math.min(1.0, power));

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