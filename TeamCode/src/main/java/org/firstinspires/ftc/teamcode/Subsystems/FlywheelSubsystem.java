package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.GlobalConstants;

public class FlywheelSubsystem {

    private DcMotorEx motorSH;
    private DcMotorEx motorHS;

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

    public void start() {
        timer.reset();
        lastErrorTPS = 0;
        integralSum = 0;
    }

    public void update(double targetVelocityRPM, boolean isEmergencyBrake, boolean isActiveSpooling) {
        // [1] 动态容差计算 - 引用 GlobalConstants
        double dynamicTolerance;
        if (targetVelocityRPM <= GlobalConstants.FLYWHEEL_RPM_MIN) {
            dynamicTolerance = GlobalConstants.FLYWHEEL_MAX_TOLERANCE;
        } else if (targetVelocityRPM >= GlobalConstants.FLYWHEEL_RPM_MAX) {
            dynamicTolerance = GlobalConstants.FLYWHEEL_MIN_TOLERANCE;
        } else {
            double ratio = (targetVelocityRPM - GlobalConstants.FLYWHEEL_RPM_MIN) / (GlobalConstants.FLYWHEEL_RPM_MAX - GlobalConstants.FLYWHEEL_RPM_MIN);
            dynamicTolerance = GlobalConstants.FLYWHEEL_MAX_TOLERANCE - ratio * (GlobalConstants.FLYWHEEL_MAX_TOLERANCE - GlobalConstants.FLYWHEEL_MIN_TOLERANCE);
        }

        double currentVelTPS = motorSH.getVelocity();
        double targetVelTPS = (targetVelocityRPM * GlobalConstants.FLYWHEEL_TICKS_PER_REV) / 60.0;
        currentRPM = (currentVelTPS * 60.0) / GlobalConstants.FLYWHEEL_TICKS_PER_REV;
        double errorRPM = targetVelocityRPM - currentRPM;

        // [2] 飞轮就绪状态机
        if (!isActiveSpooling) {
            isFlywheelReady = false;
        } else {
            if (!isFlywheelReady) {
                if (currentRPM >= targetVelocityRPM - GlobalConstants.FLYWHEEL_SPOOL_UP_TOLERANCE) {
                    isFlywheelReady = true;
                }
            } else {
                if (currentRPM < targetVelocityRPM - dynamicTolerance) {
                    isFlywheelReady = false;
                }
            }
        }

        double dt = timer.seconds();
        timer.reset();
        if (dt == 0) dt = 1e-9;

        double errorTPS = targetVelTPS - currentVelTPS;

        // [3] 积分计算
        if (targetVelocityRPM > 100) {
            integralSum += errorTPS * dt;
        } else {
            integralSum = 0;
        }

        double maxIntegral = 0.25;
        if (GlobalConstants.FLYWHEEL_kI != 0) {
            if (integralSum > maxIntegral / GlobalConstants.FLYWHEEL_kI) integralSum = maxIntegral / GlobalConstants.FLYWHEEL_kI;
            if (integralSum < -maxIntegral / GlobalConstants.FLYWHEEL_kI) integralSum = -maxIntegral / GlobalConstants.FLYWHEEL_kI;
        }

        // [4] 微分计算
        double derivative = (errorTPS - lastErrorTPS) / dt;
        lastErrorTPS = errorTPS;

        // [5] PIDF 控制律 - 引用 GlobalConstants
        double power = (GlobalConstants.FLYWHEEL_kF * targetVelTPS) +
                (GlobalConstants.FLYWHEEL_kP * errorTPS) +
                (GlobalConstants.FLYWHEEL_kI * integralSum) +
                (GlobalConstants.FLYWHEEL_kD * derivative);

        // [6] Bang-Bang 补电
        double bangBangThreshold = Math.abs(dynamicTolerance) - (Math.abs(dynamicTolerance) / 7.0);
        activeBoost = "None";

        if (isActiveSpooling && targetVelocityRPM > 100) {
            if (errorRPM > bangBangThreshold) {
                power = 1.0;
                integralSum = 0;
                activeBoost = "Bang-Bang (极速补电)";
            }
        }

        if (isEmergencyBrake || targetVelocityRPM <= 0) {
            power = 0;
            integralSum = 0;
        }

        power = Math.max(-1.0, Math.min(1.0, power));

        motorSH.setPower(power);
        motorHS.setPower(power);
    }

    public boolean isReady() { return isFlywheelReady; }
    public double getCurrentRPM() { return currentRPM; }
    public String getActiveBoostState() { return activeBoost; }
}