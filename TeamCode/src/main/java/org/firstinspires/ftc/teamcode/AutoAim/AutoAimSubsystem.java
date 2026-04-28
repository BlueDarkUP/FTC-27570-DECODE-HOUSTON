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
import org.firstinspires.ftc.teamcode.Storage.RobotStateStorage;

@Config
public class AutoAimSubsystem {

    private DcMotorEx Turret;
    private Servo LP;
    private Servo RP;
    private VoltageSensor battery;
    private HardwareMap hardwareMap;

    public static double TURRET_kP = 30.0;
    public static double TURRET_kI = 0.0;
    public static double TURRET_kD = 0.0;
    public static double TURRET_kF = 0.0;
    public static double TURRET_kV = 0.001394;
    public static double TURRET_kS = 0.03;
    public static double TURRET_kA = 0.000069;
    public static double TURRET_LATENCY = 0.012;
    public static double TURRET_MAX_POWER = 1.0;
    public static double TURRET_FILTER_ALPHA = 0.8;
    public static double TURRET_VEL_FILTER_ALPHA = 0.8;
    public static double TURRET_CMD_ACCEL_FILTER_ALPHA = 0.15;

    public static double TURRET_kLinearBraking = 0.026905;
    public static double TURRET_kQuadraticFriction = 0.000078;
    public static double TUNING_VOLTAGE = 13.84;
    public static double CHASSIS_VEL_FILTER_ALPHA = 0.3;
    public static double CHASSIS_ACCEL_FILTER_ALPHA = 0.4;
    public static double BRAKE_PREDICTION_WEIGHT = 0.55;
    public static double FORWARD_BRAKE_DECEL = 33.09567766;
    public static double LATERAL_BRAKE_DECEL = 52.88478403;
    public static double OMEGA_FILTER_ALPHA = 0.7;
    public static double HEADING_CORRECTION_ALPHA = 0.5;

    private PIDFController turretPIDF;

    private double initialTurretOffset = 0.0;
    private double filteredTurretRelAngle = 0.0;
    private boolean isTurretFilterInitialized = false;
    private double lastTurretRelAngle = 0.0;
    private double filteredTurretVel = 0.0;
    private double lastTargetVel = 0.0;
    private double filteredTargetAccel = 0.0;
    private long lastTime = 0;
    private long lastVoltageReadTime = 0;
    private double currentBatteryVoltage = 12.0;
    private boolean isCurrentlyUnwinding = false;
    private boolean isChassisFilterInitialized = false;
    private double filteredForward = 0.0;
    private double filteredLateral = 0.0;
    private double filteredForwardAccel = 0.0;
    private double filteredLateralAccel = 0.0;
    private double lastFilteredForward = 0.0;
    private double lastFilteredLateral = 0.0;
    private double filteredRobotOmega = 0.0;
    private double smoothHeading = 0.0;
    private boolean isSmoothHeadingInit = false;

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
        if (RobotStateStorage.isAutoDataValid) {
            initialTurretOffset = RobotStateStorage.turretAngleDeg;
        }
        else {
            initialTurretOffset = 0;
        }
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
    private double predictAxisAccel(double velocity, double measuredAccel, boolean isBraking, double brakeDecel) {
        if (!isBraking || Math.abs(velocity) <= 0.0001) {
            return measuredAccel;
        }
        double brakeAccel = -Math.signum(velocity) * brakeDecel;
        return measuredAccel * (1.0 - BRAKE_PREDICTION_WEIGHT) + brakeAccel * BRAKE_PREDICTION_WEIGHT;
    }
    private double stopAtZero(double currentValue, double predictedValue) {
        if (currentValue > 0.0 && predictedValue < 0.0) return 0.0;
        if (currentValue < 0.0 && predictedValue > 0.0) return 0.0;
        return predictedValue;
    }
    public TurretCommand update(
            double robotX, double robotY, double globalVx, double globalVy,
            double currentHeadingDeg, double robotAngularVelocityDeg,
            double targetX, double targetY,
            boolean isManualMode, double manualDist, boolean isClimbing,
            boolean isShootOnTheMove, boolean isBraking, double yawOffset) {

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
        double rawTurretRelAngle = (currentTurretTicks / GlobalConstants.TURRET_TICKS_PER_REV) * 360.0 + initialTurretOffset;

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

            double ksSign = Math.abs(finalTargetVel) > 0.5 ? Math.signum(finalTargetVel) : 0.0;
            double turretPower = (filteredTargetAccel * TURRET_kA)
                    + (finalTargetVel * TURRET_kV)
                    + (ksSign * TURRET_kS);

            double voltageCompensationRatio = TUNING_VOLTAGE / currentBatteryVoltage;
            turretPower *= voltageCompensationRatio;
            turretPower = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, turretPower));

            Turret.setPower(turretPower);
            setPitchServos(command.targetPitch);

            return command;
        }

        if (!isSmoothHeadingInit) {
            smoothHeading = currentHeadingDeg;
            filteredRobotOmega = robotAngularVelocityDeg;
            isSmoothHeadingInit = true;
        } else {
            filteredRobotOmega += OMEGA_FILTER_ALPHA * (robotAngularVelocityDeg - filteredRobotOmega);
            smoothHeading += filteredRobotOmega * dt;
            double headingError = currentHeadingDeg - smoothHeading;
            while (headingError > 180) headingError -= 360;
            while (headingError < -180) headingError += 360;
            smoothHeading += HEADING_CORRECTION_ALPHA * headingError;
            while (smoothHeading > 180) smoothHeading -= 360;
            while (smoothHeading < -180) smoothHeading += 360;
        }
        double headingRad = Math.toRadians(smoothHeading);
        double cosH = Math.cos(headingRad);
        double sinH = Math.sin(headingRad);

        double rawForward = globalVx * cosH + globalVy * sinH;
        double rawLateral = -globalVx * sinH + globalVy * cosH;

        if (!isChassisFilterInitialized || !isShootOnTheMove) {
            filteredForward = rawForward;
            filteredLateral = rawLateral;
            filteredForwardAccel = 0.0;
            filteredLateralAccel = 0.0;
            lastFilteredForward = rawForward;
            lastFilteredLateral = rawLateral;
            isChassisFilterInitialized = true;
        } else {
            filteredForward += CHASSIS_VEL_FILTER_ALPHA * (rawForward - filteredForward);
            filteredLateral += CHASSIS_VEL_FILTER_ALPHA * (rawLateral - filteredLateral);
            if (dt > 0.0001) {
                double rawForwardAccel = (filteredForward - lastFilteredForward) / dt;
                double rawLateralAccel = (filteredLateral - lastFilteredLateral) / dt;
                filteredForwardAccel += CHASSIS_ACCEL_FILTER_ALPHA * (rawForwardAccel - filteredForwardAccel);
                filteredLateralAccel += CHASSIS_ACCEL_FILTER_ALPHA * (rawLateralAccel - filteredLateralAccel);
            }

            lastFilteredForward = filteredForward;
            lastFilteredLateral = filteredLateral;
        }
        double predictedForwardAccel = 0.0;
        double predictedLateralAccel = 0.0;
        if (isShootOnTheMove) {
            predictedForwardAccel = predictAxisAccel(filteredForward, filteredForwardAccel, isBraking, FORWARD_BRAKE_DECEL);
            predictedLateralAccel = predictAxisAccel(filteredLateral, filteredLateralAccel, isBraking, LATERAL_BRAKE_DECEL);
        }
        double effectiveFieldVx = filteredForward * cosH - filteredLateral * sinH;
        double effectiveFieldVy = filteredForward * sinH + filteredLateral * cosH;

        if (isManualMode) {
            double manualRpm = AimCalculator.interpolate(manualDist, 1);
            double manualPitch = AimCalculator.interpolate(manualDist, 2);
            aimResult = new AimCalculator.AimResult(manualDist, smoothHeading, manualRpm, manualPitch, 0.0);
        } else {
            double currentDistToTarget = Math.hypot(targetX - robotX, targetY - robotY);
            double dynamicFlightTime = (currentDistToTarget >= AimCalculator.FAR_DIST_THRESHOLD) ?
                    AimCalculator.FAR_FLIGHT_TIME :
                    AimCalculator.CONSTANT_FLIGHT_TIME;

            double latency = AimCalculator.MECHANICAL_SHOOT_DELAY;

            double releaseForward = filteredForward + predictedForwardAccel * latency;
            double releaseLateral = filteredLateral + predictedLateralAccel * latency;
            if (isBraking) {
                releaseForward = stopAtZero(filteredForward, releaseForward);
                releaseLateral = stopAtZero(filteredLateral, releaseLateral);
            }

            double avgForward = 0.5 * (filteredForward + releaseForward);
            double avgLateral = 0.5 * (filteredLateral + releaseLateral);

            double releaseX = robotX + (avgForward * cosH - avgLateral * sinH) * latency;
            double releaseY = robotY + (avgForward * sinH + avgLateral * cosH) * latency;

            double releaseFieldVx = releaseForward * cosH - releaseLateral * sinH;
            double releaseFieldVy = releaseForward * sinH + releaseLateral * cosH;

            double futureX = releaseX + (releaseFieldVx * dynamicFlightTime);
            double futureY = releaseY + (releaseFieldVy * dynamicFlightTime);

            aimResult = AimCalculator.solveAim(futureX, futureY, targetX, targetY, dynamicFlightTime);
        }

        if (aimResult != null) {
            command.hasTarget = true;
            command.targetRpm = aimResult.rpm;
            command.targetPitch = aimResult.pitch;
            command.targetDist = aimResult.dist;
            double calculatedTolerance;
            if (aimResult.dist <= 20.0) {
                calculatedTolerance = 30.0;
            } else if (aimResult.dist >= 90.0) {
                calculatedTolerance = 5.0;
            } else {
                calculatedTolerance = 30.0 + (5.0 - 30.0) / (90.0 - 20.0) * (aimResult.dist - 20.0);
            }
            command.currentTolerance = calculatedTolerance;

            double dx = targetX - robotX;
            double dy = targetY - robotY;
            double distSq = dx * dx + dy * dy;
            double translationalOmegaDeg = 0.0;
            if (distSq > 0.001) {
                double omegaRad = (-dx * effectiveFieldVy + dy * effectiveFieldVx) / distSq;
                translationalOmegaDeg = Math.toDegrees(omegaRad);
            }
            double compensatedTargetAbsAngle = aimResult.algYaw + yawOffset + (translationalOmegaDeg * TURRET_LATENCY);
            double currentTurretAbsAngle = smoothHeading + filteredTurretRelAngle;
            double error = compensatedTargetAbsAngle - currentTurretAbsAngle;

            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            double targetTurretRelAngle = filteredTurretRelAngle + error;

            if (targetTurretRelAngle > 175.0) {
                targetTurretRelAngle -= 360.0;
                isCurrentlyUnwinding = true;
            } else if (targetTurretRelAngle < -210.0) {
                targetTurretRelAngle += 360.0;
                isCurrentlyUnwinding = true;
            }

            if (isCurrentlyUnwinding) {
                if (Math.abs(targetTurretRelAngle - filteredTurretRelAngle) < 15.0 && Math.abs(filteredTurretVel) < 20.0) {
                    isCurrentlyUnwinding = false;
                }
            }
            command.isUnwinding = isCurrentlyUnwinding;
            command.isAimLocked = Math.abs(error) <= command.currentTolerance;

            double brakingDist = 0.0;
            double predictedRelAngle = filteredTurretRelAngle;
            if (command.isUnwinding) {
                brakingDist = (TURRET_kLinearBraking * Math.abs(filteredTurretVel))
                        + (TURRET_kQuadraticFriction * filteredTurretVel * filteredTurretVel);
                predictedRelAngle = filteredTurretRelAngle + (Math.signum(filteredTurretVel) * brakingDist);
            }

            double pidOutputVel = turretPIDF.calculate(predictedRelAngle, targetTurretRelAngle);
            double feedforwardVel = -filteredRobotOmega + translationalOmegaDeg;
            double finalTargetVel = pidOutputVel + feedforwardVel;

            double rawTargetAccel = 0.0;
            if (dt > 0.0001) rawTargetAccel = (finalTargetVel - lastTargetVel) / dt;
            lastTargetVel = finalTargetVel;

            filteredTargetAccel = (TURRET_CMD_ACCEL_FILTER_ALPHA * rawTargetAccel) + ((1.0 - TURRET_CMD_ACCEL_FILTER_ALPHA) * filteredTargetAccel);

            double ksSign = Math.abs(finalTargetVel) > 0.5 ? Math.signum(finalTargetVel) : 0.0;
            double turretPower = (filteredTargetAccel * TURRET_kA)
                    + (finalTargetVel * TURRET_kV)
                    + (ksSign * TURRET_kS);

            double voltageCompensationRatio = TUNING_VOLTAGE / currentBatteryVoltage;
            turretPower *= voltageCompensationRatio;
            turretPower = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, turretPower));

            Turret.setPower(turretPower);
            setPitchServos(command.targetPitch);

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Turret/Error", error);
            packet.put("Turret/Tolerance", command.currentTolerance);
            packet.put("Turret/YawOffset", yawOffset);
            packet.put("Turret/IsLocked", command.isAimLocked);
            packet.put("Predict/Braking", isBraking);
            packet.put("Predict/FilteredForward", filteredForward);
            packet.put("Predict/FilteredLateral", filteredLateral);
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
        isCurrentlyUnwinding = false;
        isSmoothHeadingInit = false;
    }
}