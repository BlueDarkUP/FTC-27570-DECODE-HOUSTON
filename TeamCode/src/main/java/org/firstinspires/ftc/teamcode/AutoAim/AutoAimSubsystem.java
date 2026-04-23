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
    public static double TURRET_kV = 0.0012;
    public static double TURRET_kS = 0;
    public static double TURRET_kA = 0.000109;
    public static double TURRET_LATENCY = 0.012;
    public static double TURRET_MAX_POWER = 1.0;
    public static double TURRET_FILTER_ALPHA = 0.8;
    public static double TURRET_VEL_FILTER_ALPHA = 0.8;
    public static double TURRET_kLinearBraking = 0.008500;
    public static double TURRET_kQuadraticFriction = 0.000098;
    public static double TUNING_VOLTAGE = 13.04;
    public static double CHASSIS_VEL_DEADBAND = 10;
    public static double CHASSIS_VEL_FILTER_ALPHA = 0.8;
    public static double CHASSIS_ACCEL_FILTER_ALPHA = 0.025;
    public static double CHASSIS_ACCEL_DEADBAND = 2.0;
    public static double MAX_CHASSIS_ACCEL = 150.0;
    public static double TURRET_CMD_ACCEL_FILTER_ALPHA = 0.015;

    private PIDFController turretPIDF;

    private double filteredTurretRelAngle = 0.0;
    private boolean isTurretFilterInitialized = false;
    private double lastTurretRelAngle = 0.0;
    private double filteredTurretVel = 0.0;
    private double lastTargetVel = 0.0;
    private double filteredTargetAccel = 0.0;
    private long lastTime = 0;
    private long lastVoltageReadTime = 0;
    private double currentBatteryVoltage = 12.0;

    private boolean isChassisFilterInitialized = false;
    private double filteredGlobalVx = 0.0;
    private double filteredGlobalVy = 0.0;
    private double lastGlobalVx = 0.0;
    private double lastGlobalVy = 0.0;
    private double filteredAx = 0.0;
    private double filteredAy = 0.0;

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
            boolean isManualMode, double manualDist, boolean isClimbing,
            boolean isShootOnTheMove) {

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
            filteredTargetAccel = 0.0;
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

            double rawTargetAccel = 0.0;
            if (dt > 0.0001) rawTargetAccel = (finalTargetVel - lastTargetVel) / dt;
            lastTargetVel = finalTargetVel;
            filteredTargetAccel = (TURRET_CMD_ACCEL_FILTER_ALPHA * rawTargetAccel) + ((1.0 - TURRET_CMD_ACCEL_FILTER_ALPHA) * filteredTargetAccel);

            double turretPower = (filteredTargetAccel * TURRET_kA)
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
            lastGlobalVx = globalVx;
            lastGlobalVy = globalVy;
            filteredAx = 0.0;
            filteredAy = 0.0;
            isChassisFilterInitialized = true;
        } else {
            double rawAx = (dt > 0.0001) ? (globalVx - lastGlobalVx) / dt : 0.0;
            double rawAy = (dt > 0.0001) ? (globalVy - lastGlobalVy) / dt : 0.0;

            filteredAx = (CHASSIS_ACCEL_FILTER_ALPHA * rawAx) + ((1.0 - CHASSIS_ACCEL_FILTER_ALPHA) * filteredAx);
            filteredAy = (CHASSIS_ACCEL_FILTER_ALPHA * rawAy) + ((1.0 - CHASSIS_ACCEL_FILTER_ALPHA) * filteredAy);

            filteredGlobalVx = (CHASSIS_VEL_FILTER_ALPHA * globalVx) + ((1.0 - CHASSIS_VEL_FILTER_ALPHA) * filteredGlobalVx);
            filteredGlobalVy = (CHASSIS_VEL_FILTER_ALPHA * globalVy) + ((1.0 - CHASSIS_VEL_FILTER_ALPHA) * filteredGlobalVy);

            lastGlobalVx = globalVx;
            lastGlobalVy = globalVy;
        }

        double effectiveVx = isShootOnTheMove ? filteredGlobalVx : 0.0;
        double effectiveVy = isShootOnTheMove ? filteredGlobalVy : 0.0;

        double effectiveAx = 0.0;
        double effectiveAy = 0.0;
        if (isShootOnTheMove) {
            if (Math.abs(filteredAx) > CHASSIS_ACCEL_DEADBAND) {
                effectiveAx = Math.max(-MAX_CHASSIS_ACCEL, Math.min(MAX_CHASSIS_ACCEL, filteredAx));
            }
            if (Math.abs(filteredAy) > CHASSIS_ACCEL_DEADBAND) {
                effectiveAy = Math.max(-MAX_CHASSIS_ACCEL, Math.min(MAX_CHASSIS_ACCEL, filteredAy));
            }
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
            double futureX = robotX + (effectiveVx * totalTime) + (0.5 * effectiveAx * totalTime * totalTime);
            double futureY = robotY + (effectiveVy * totalTime) + (0.5 * effectiveAy * totalTime * totalTime);
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
                double omegaRad = (-dx * effectiveVy + dy * effectiveVx) / distSq;
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

            double rawTargetAccel = 0.0;
            if (dt > 0.0001) rawTargetAccel = (finalTargetVel - lastTargetVel) / dt;
            lastTargetVel = finalTargetVel;

            filteredTargetAccel = (TURRET_CMD_ACCEL_FILTER_ALPHA * rawTargetAccel) + ((1.0 - TURRET_CMD_ACCEL_FILTER_ALPHA) * filteredTargetAccel);

            double turretPower = (filteredTargetAccel * TURRET_kA)
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
            packet.put("Predict/AccelX", effectiveAx);
            packet.put("Predict/AccelY", effectiveAy);
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