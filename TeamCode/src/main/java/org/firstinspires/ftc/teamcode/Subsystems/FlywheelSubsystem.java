package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class FlywheelSubsystem {

    private DcMotorEx motorSH;
    private DcMotorEx motorHS;

    public static final double IDLE_VELOCITY = 3000.0;
    private final double RPM_LOWER_BOUND = 3000.0;
    private final double RPM_UPPER_BOUND = 5050.0;
    private final double MAX_TOLERANCE = 1000;
    private final double MIN_TOLERANCE = 1000;
    private final double SPOOL_UP_TOLERANCE = 100.0;

    private final double kP = 0.011, kI = 0.0004, kD = 0.00000023, kF = 0.00033;
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

        motorSH.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorHS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /**
     * 在主程序的 waitForStart() 之后调用，重置计时器防止首个循环时间 dt 过大
     */
    public void start() {
        timer.reset();
        lastErrorTPS = 0;
        integralSum = 0;
    }

    /**
     * 飞轮核心更新与PID控制函数
     *
     * @param targetVelocityRPM 目标RPM
     * @param isEmergencyBrake  是否处于紧急刹车模式
     * @param isActiveSpooling  是否处于主动蓄力/射击状态 (用于判断是否需要Bang-Bang和判断就绪)
     */
    public void update(double targetVelocityRPM, boolean isEmergencyBrake, boolean isActiveSpooling) {
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
        timer.reset();
        if (dt == 0) dt = 1e-9;

        double errorTPS = targetVelTPS - currentVelTPS;

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

        double derivative = (errorTPS - lastErrorTPS) / dt;
        lastErrorTPS = errorTPS;

        double power = (kF * targetVelTPS) + (kP * errorTPS) + (kI * integralSum) + (kD * derivative);

        double bangBangThreshold = Math.max(50.0, Math.abs(dynamicTolerance) - (Math.abs(dynamicTolerance) / 7.0));
        activeBoost = "None";

        if (isActiveSpooling && targetVelocityRPM > 100) {
            if (errorRPM > bangBangThreshold) {
                power = 1.0;
                integralSum = 0;
                activeBoost = "Bang-Bang (极速补电)";
            }
        }

        if (targetVelocityRPM <= 0 || isEmergencyBrake) {
            power = 0;
            integralSum = 0;
        }

        power = Math.max(0.0, Math.min(1.0, power));
        motorSH.setPower(power);
        motorHS.setPower(power);
    }

    // --- 状态读取接口 ---

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