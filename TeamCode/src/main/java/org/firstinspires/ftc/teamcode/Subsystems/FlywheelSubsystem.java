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

    public static final double IDLE_VELOCITY_MIN = 3000.0;

    private final double RPM_LOWER_BOUND = 4250.0;
    private final double RPM_UPPER_BOUND = 4850.0;
    private final double MAX_TOLERANCE = 1000;
    private final double MIN_TOLERANCE = 50;
    private final double SPOOL_UP_TOLERANCE = 50.0;

    // PIDF 参数
    public static double kP = 0.025;
    public static double kI = 0.000;
    public static double kD = 0.000;
    public static double kV = 0.0002;
    public static double kS = 0.03;

    private final double TICKS_PER_REV = 28.0;

    private ElapsedTime timer = new ElapsedTime();
    private double lastErrorTPS = 0;
    private double integralSum = 0;

    private boolean isFlywheelReady = false;
    private double currentRPM = 0.0;
    private String activeBoost = "None (纯PIDF)";

    public FlywheelSubsystem(HardwareMap hardwareMap) {
        motorSH = hardwareMap.get(DcMotorEx.class, "SH");
        motorHS = hardwareMap.get(DcMotorEx.class, "HS");

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

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

        double currentVoltage = batteryVoltageSensor.getVoltage();

        double feedforward = 0.0;
        if (targetVelTPS > 0) {
            feedforward = kS * Math.signum(targetVelTPS) + kV * targetVelTPS;
        }

        double pid = (kP * errorTPS) + (kI * integralSum) + (kD * derivative);

        double basePower = feedforward + pid;
        double power = basePower * (12.9 / currentVoltage);

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