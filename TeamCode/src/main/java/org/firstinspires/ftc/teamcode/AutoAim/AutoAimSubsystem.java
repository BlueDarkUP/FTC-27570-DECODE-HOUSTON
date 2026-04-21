package org.firstinspires.ftc.teamcode.AutoAim;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.GlobalConstants;

@Config
public class AutoAimSubsystem {

    private DcMotorEx Turret;
    private Servo LP;
    private Servo RP;
    private VoltageSensor battery;
    private HardwareMap hardwareMap;

    public static double TURRET_kP = 33.0;
    public static double TURRET_kI = 0.0;
    public static double TURRET_kD = 0.0;
    public static double TURRET_kF = 0.0;
    public static double TURRET_kV = 0.001201;
    public static double TURRET_kS = 0.0;
    public static double TURRET_kA = 0.000113;
    public static double TURRET_LATENCY = 0.01;
    public static double TURRET_MAX_POWER = 1.0;
    public static double TURRET_FILTER_ALPHA = 0.9;
    public static double TURRET_VEL_FILTER_ALPHA = 0.9;
    public static double TURRET_kLinearBraking = 0.008500;
    public static double TURRET_kQuadraticFriction = 0.000098;
    public static double TUNING_VOLTAGE = 13.04;
    public static double CHASSIS_VEL_DEADBAND = 5;
    public static double CHASSIS_VEL_FILTER_ALPHA = 0.8;

    private PIDFController turretPIDF;

    private double filteredTurretRelAngle = 0.0;
    private boolean isTurretFilterInitialized = false;
    private double lastTurretRelAngle = 0.0;
    private double filteredTurretVel = 0.0;
    private double lastTargetVel = 0.0;
    private long lastTime = 0;
    private long lastVoltageReadTime = 0;
    private double currentBatteryVoltage = 12.0;

    private boolean isChassisFilterInitialized = false;
    private double filteredGlobalVx = 0.0;
    private double filteredGlobalVy = 0.0;

    public static class TurretCommand {
        public boolean hasTarget = false;
        public double targetRpm = 0.0;
        public double targetPitch = 0.0;
        public boolean isAimLocked = false;
        public boolean isUnwinding = false;
        public double currentTolerance = 1.0;
        public double targetDist = 0.0;
    }

    public AutoAimSubsystem(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        Turret = hardwareMap.get(DcMotorEx.class, "Turret");
        Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        LP = hardwareMap.get(Servo.class, "LP");
        RP = hardwareMap.get(Servo.class, "RP");
        battery = hardwareMap.voltageSensor.iterator().next();
        currentBatteryVoltage = getBatteryVoltage();
        turretPIDF = new PIDFController(TURRET_kP, TURRET_kI, TURRET_kD, TURRET_kF);
        setPitchServos(0.7);
    }

    private double getBatteryVoltage() {
        double maxVoltage = 0;
        for (VoltageSensor sensor : this.hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > maxVoltage) maxVoltage = v;
        }
        return Math.max(8.0, maxVoltage > 0 ? maxVoltage : TUNING_VOLTAGE);
    }

    public double getCurrentBatteryVoltage() {
        return currentBatteryVoltage;
    }

    private void setPitchServos(double targetPitch) {
        double clampedLP = Math.max(GlobalConstants.PITCH_LP_DOWN, Math.min(GlobalConstants.PITCH_LP_UP, targetPitch));
        double proportion = (clampedLP - GlobalConstants.PITCH_LP_DOWN) / (GlobalConstants.PITCH_LP_UP - GlobalConstants.PITCH_LP_DOWN);
        double calculatedRP = GlobalConstants.PITCH_RP_DOWN + proportion * (GlobalConstants.PITCH_RP_UP - GlobalConstants.PITCH_RP_DOWN);
        calculatedRP = Math.max(0.0, Math.min(1.0, calculatedRP));
        LP.setPosition(clampedLP);
        RP.setPosition(calculatedRP);
    }

    public TurretCommand update(
            double robotX, double robotY, double globalVx, double globalVy,
            double currentHeadingDeg, double robotAngularVelocityDeg,
            double targetX, double targetY,
            boolean isManualMode, double manualDist, boolean isClimbing) {

        turretPIDF.setPIDF(TURRET_kP, TURRET_kI, TURRET_kD, TURRET_kF);
        TurretCommand command = new TurretCommand();
        AimCalculator.AimResult aimResult;

        long currentTimeForVolts = System.nanoTime();
        if (currentTimeForVolts - lastVoltageReadTime > 250_000_000L) {
            currentBatteryVoltage = getBatteryVoltage();
            lastVoltageReadTime = currentTimeForVolts;
        }

        long currentTimeNanos = System.nanoTime();
        double dt = (lastTime == 0) ? 0.0001 : (currentTimeNanos - lastTime) / 1e9;
        lastTime = currentTimeNanos;

        double currentTurretTicks = Turret.getCurrentPosition();
        double rawTurretRelAngle = (currentTurretTicks / GlobalConstants.TURRET_TICKS_PER_REV) * 360.0;

        if (!isTurretFilterInitialized) {
            filteredTurretRelAngle = rawTurretRelAngle;
            lastTurretRelAngle = rawTurretRelAngle;
            filteredTurretVel = 0.0;
            lastTargetVel = 0.0;
            isTurretFilterInitialized = true;
        } else {
            filteredTurretRelAngle = (TURRET_FILTER_ALPHA * rawTurretRelAngle) + ((1.0 - TURRET_FILTER_ALPHA) * filteredTurretRelAngle);
            if (dt > 0.0001) {
                double rawVel = (rawTurretRelAngle - lastTurretRelAngle) / dt;
                filteredTurretVel = (TURRET_VEL_FILTER_ALPHA * rawVel) + ((1.0 - TURRET_VEL_FILTER_ALPHA) * filteredTurretVel);
            }
        }
        lastTurretRelAngle = rawTurretRelAngle;

        if (isClimbing) {
            command.hasTarget = false;
            command.targetRpm = 0.0;
            command.targetPitch = 0.0;
            command.isUnwinding = false;

            double targetTurretRelAngle = 0.0;
            double predictedRelAngle = filteredTurretRelAngle;

            double pidOutputVel = turretPIDF.calculate(predictedRelAngle, targetTurretRelAngle);
            double finalTargetVel = pidOutputVel;

            double targetAccel = 0.0;
            if (dt > 0.0001) targetAccel = (finalTargetVel - lastTargetVel) / dt;
            lastTargetVel = finalTargetVel;

            double turretPower = (targetAccel * TURRET_kA)
                    + (finalTargetVel * TURRET_kV)
                    + (Math.signum(finalTargetVel) * TURRET_kS);

            double voltageCompensationRatio = TUNING_VOLTAGE / currentBatteryVoltage;
            turretPower *= voltageCompensationRatio;
            turretPower = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, turretPower));

            Turret.setPower(turretPower);
            setPitchServos(command.targetPitch);

            return command;
        }

        double currentSpeed = Math.hypot(globalVx, globalVy);
        if (currentSpeed < CHASSIS_VEL_DEADBAND) {
            globalVx = 0.0;
            globalVy = 0.0;
        }

        if (!isChassisFilterInitialized) {
            filteredGlobalVx = globalVx;
            filteredGlobalVy = globalVy;
            isChassisFilterInitialized = true;
        } else {
            filteredGlobalVx = (CHASSIS_VEL_FILTER_ALPHA * globalVx) + ((1.0 - CHASSIS_VEL_FILTER_ALPHA) * filteredGlobalVx);
            filteredGlobalVy = (CHASSIS_VEL_FILTER_ALPHA * globalVy) + ((1.0 - CHASSIS_VEL_FILTER_ALPHA) * filteredGlobalVy);
        }

        if (isManualMode) {
            double manualRpm = AimCalculator.interpolate(manualDist, 1);
            double manualPitch = AimCalculator.interpolate(manualDist, 2);
            aimResult = new AimCalculator.AimResult(manualDist, currentHeadingDeg, manualRpm, manualPitch, 0.0);
        } else {
            double currentDistToTarget = Math.hypot(targetX - robotX, targetY - robotY);
            double dynamicFlightTime = (currentDistToTarget >= AimCalculator.FAR_DIST_THRESHOLD) ?
                    AimCalculator.FAR_FLIGHT_TIME :
                    AimCalculator.CONSTANT_FLIGHT_TIME;

            double totalTime = dynamicFlightTime + AimCalculator.MECHANICAL_SHOOT_DELAY;

            double futureX = robotX + (filteredGlobalVx * totalTime);
            double futureY = robotY + (filteredGlobalVy * totalTime);
            aimResult = AimCalculator.solveAim(futureX, futureY, targetX, targetY, dynamicFlightTime);
        }

        if (aimResult != null) {
            command.hasTarget = true;
            command.targetRpm = aimResult.rpm;
            command.targetPitch = aimResult.pitch;
            command.targetDist = aimResult.dist;

            double calculatedTolerance = 25.0 + (1.5 - 25.0) / (150.0 - 20.0) * (aimResult.dist - 20.0);
            command.currentTolerance = Math.max(1.5, Math.min(25.0, calculatedTolerance));

            double dx = targetX - robotX;
            double dy = targetY - robotY;
            double distSq = dx * dx + dy * dy;
            double translationalOmegaDeg = 0.0;
            if (distSq > 0.001) {
                double omegaRad = (-dx * filteredGlobalVy + dy * filteredGlobalVx) / distSq;
                translationalOmegaDeg = Math.toDegrees(omegaRad);
            }

            double compensatedTargetAbsAngle = aimResult.algYaw + (translationalOmegaDeg * TURRET_LATENCY);
            double currentTurretAbsAngle = currentHeadingDeg + filteredTurretRelAngle;
            double error = compensatedTargetAbsAngle - currentTurretAbsAngle;

            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            double targetTurretRelAngle = filteredTurretRelAngle + error;

            if (targetTurretRelAngle > 175.0) {
                targetTurretRelAngle -= 360.0;
                command.isUnwinding = true;
            } else if (targetTurretRelAngle < -210.0) {
                targetTurretRelAngle += 360.0;
                command.isUnwinding = true;
            } else {
                command.isUnwinding = false;
            }

            command.isAimLocked = Math.abs(error) <= command.currentTolerance;

            double brakingDist = 0.0;
            double predictedRelAngle = filteredTurretRelAngle;
            if (command.isUnwinding) {
                brakingDist = (TURRET_kLinearBraking * Math.abs(filteredTurretVel))
                        + (TURRET_kQuadraticFriction * filteredTurretVel * filteredTurretVel);
                predictedRelAngle = filteredTurretRelAngle + (Math.signum(filteredTurretVel) * brakingDist);
            }

            double pidOutputVel = turretPIDF.calculate(predictedRelAngle, targetTurretRelAngle);
            double feedforwardVel = -robotAngularVelocityDeg + translationalOmegaDeg;
            double finalTargetVel = pidOutputVel + feedforwardVel;

            double targetAccel = 0.0;
            if (dt > 0.0001) targetAccel = (finalTargetVel - lastTargetVel) / dt;
            lastTargetVel = finalTargetVel;

            double turretPower = (targetAccel * TURRET_kA)
                    + (finalTargetVel * TURRET_kV)
                    + (Math.signum(finalTargetVel) * TURRET_kS);

            double voltageCompensationRatio = TUNING_VOLTAGE / currentBatteryVoltage;
            turretPower *= voltageCompensationRatio;
            turretPower = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, turretPower));

            Turret.setPower(turretPower);
            setPitchServos(command.targetPitch);

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Turret/Error", error);
            packet.put("Turret/IsLocked", command.isAimLocked);
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

        } else {
            Turret.setPower(0);
        }
        return command;
    }

    public void stop() {
        Turret.setPower(0);
        isTurretFilterInitialized = false;
        isChassisFilterInitialized = false;
    }
}