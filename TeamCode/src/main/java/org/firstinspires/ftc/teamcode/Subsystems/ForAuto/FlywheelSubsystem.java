package org.firstinspires.ftc.teamcode.Subsystems.ForAuto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GlobalConstants;

public class FlywheelSubsystem {
    private DcMotorEx motorSH;
    private DcMotorEx motorHS;

    private final double FW_MAX_TOLERANCE = 1000;
    private final double FW_MIN_TOLERANCE = 1000;
    private final double FW_SPOOL_UP_TOLERANCE = 100.0;

    private final double FW_kP = 0.011;
    private final double FW_kI = 0.0004;
    private final double FW_kD = 0.00000023;
    private final double FW_kF = 0.00033;

    private double integralSum = 0;
    private double lastErrorTPS = 0;
    private boolean isReady = false;
    private ElapsedTime timer;

    private double targetRPM = 0.0;
    private double currentRPM = 0.0;

    public FlywheelSubsystem(HardwareMap hardwareMap) {
        motorSH = hardwareMap.get(DcMotorEx.class, "SH");
        motorHS = hardwareMap.get(DcMotorEx.class, "HS");

        motorSH.setDirection(DcMotorSimple.Direction.FORWARD);
        motorHS.setDirection(DcMotorSimple.Direction.REVERSE);

        motorSH.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorHS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorSH.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorHS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        timer = new ElapsedTime();
        resetTimer();
    }

    public void setTargetRPM(double rpm) {
        this.targetRPM = rpm;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getCurrentRPM() {
        return currentRPM;
    }

    public boolean isReady() {
        return isReady;
    }

    public void resetTimer() {
        timer.reset();
        integralSum = 0;
        lastErrorTPS = 0;
    }

    public void update() {
        double currentVelTPS = motorSH.getVelocity();
        double targetVelTPS = (targetRPM * GlobalConstants.FLYWHEEL_TICKS_PER_REV) / 60.0;

        currentRPM = (currentVelTPS * 60.0) / GlobalConstants.FLYWHEEL_TICKS_PER_REV;
        double errorRPM = targetRPM - currentRPM;

        double dynamicTolerance;
        if (targetRPM <= GlobalConstants.FLYWHEEL_RPM_MIN) {
            dynamicTolerance = FW_MAX_TOLERANCE;
        } else if (targetRPM >= GlobalConstants.FLYWHEEL_RPM_MAX) {
            dynamicTolerance = FW_MIN_TOLERANCE;
        } else {
            double ratio = (targetRPM - GlobalConstants.FLYWHEEL_RPM_MIN) / (GlobalConstants.FLYWHEEL_RPM_MAX - GlobalConstants.FLYWHEEL_RPM_MIN);
            dynamicTolerance = FW_MAX_TOLERANCE - ratio * (FW_MAX_TOLERANCE - FW_MIN_TOLERANCE);
        }

        if (targetRPM <= 100) {
            isReady = false;
        } else {
            if (!isReady) {
                if (currentRPM >= targetRPM - FW_SPOOL_UP_TOLERANCE) {
                    isReady = true;
                }
            } else {
                if (currentRPM < targetRPM - dynamicTolerance) {
                    isReady = false;
                }
            }
        }

        double dt = timer.seconds();
        timer.reset();
        if (dt == 0) dt = 1e-9;

        double errorTPS = targetVelTPS - currentVelTPS;

        if (targetRPM > 100) {
            integralSum += errorTPS * dt;
        } else {
            integralSum = 0;
        }

        double maxIntegral = 0.25;
        if (FW_kI != 0) {
            if (integralSum > maxIntegral / FW_kI) integralSum = maxIntegral / FW_kI;
            if (integralSum < -maxIntegral / FW_kI) integralSum = -maxIntegral / FW_kI;
        }

        double derivative = (errorTPS - lastErrorTPS) / dt;
        lastErrorTPS = errorTPS;

        double power = (FW_kF * targetVelTPS) + (FW_kP * errorTPS) + (FW_kI * integralSum) + (FW_kD * derivative);

        double bangBangThreshold = Math.max(50.0, Math.abs(dynamicTolerance) - (Math.abs(dynamicTolerance) / 7.0));

        if (targetRPM > 100) {
            if (errorRPM > bangBangThreshold) {
                power = 1.0;
                integralSum = 0;
            }
        }

        if (targetRPM <= 0) {
            power = 0;
            integralSum = 0;
        }

        power = Math.max(0.0, Math.min(1.0, power));

        motorSH.setPower(power);
        motorHS.setPower(power);
    }

    public void stop() {
        motorSH.setPower(0);
        motorHS.setPower(0);
    }
}