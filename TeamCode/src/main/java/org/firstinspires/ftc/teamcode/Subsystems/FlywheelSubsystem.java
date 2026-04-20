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

    private final double MAX_TOLERANCE = 1000;
    private final double MIN_TOLERANCE = 1000;
    private final double SPOOL_UP_TOLERANCE = 100.0;

    public static double kP = 0.011;
    public static double kI = 0.0004;
    public static double kD = 0.00000023;
    public static double kF = 0.00033;

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
        double dynamicTolerance;
        if (targetVelocityRPM <= GlobalConstants.FLYWHEEL_RPM_MIN) {
            dynamicTolerance = MAX_TOLERANCE;
        } else if (targetVelocityRPM >= GlobalConstants.FLYWHEEL_RPM_MAX) {
            dynamicTolerance = MIN_TOLERANCE;
        } else {
            double ratio = (targetVelocityRPM - GlobalConstants.FLYWHEEL_RPM_MIN) / (GlobalConstants.FLYWHEEL_RPM_MAX - GlobalConstants.FLYWHEEL_RPM_MIN);
            dynamicTolerance = MAX_TOLERANCE - ratio * (MAX_TOLERANCE - MIN_TOLERANCE);
        }

        double currentVelTPS = motorSH.getVelocity();
        double targetVelTPS = (targetVelocityRPM * GlobalConstants.FLYWHEEL_TICKS_PER_REV) / 60.0;
        currentRPM = (currentVelTPS * 60.0) / GlobalConstants.FLYWHEEL_TICKS_PER_REV;
        double errorRPM = targetVelocityRPM - currentRPM;

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