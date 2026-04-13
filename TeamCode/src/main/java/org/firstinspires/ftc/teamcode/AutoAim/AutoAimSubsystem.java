package org.firstinspires.ftc.teamcode.AutoAim;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Driver.EchoLapse.PinpointPoseProvider;

@Config
public class AutoAimSubsystem {

    private PinpointPoseProvider robotPose;
    private PinpointPoseProvider turretPose;
    private Limelight3A ll;
    private Telemetry telemetry;
    private DcMotorEx turretMotor;

    public static double FIELD_OFFSET_X = 72.0;
    public static double FIELD_OFFSET_Y = 72.0;
    public static double TURRET_OFFSET_FWD = -1.22;
    public static double TURRET_OFFSET_LEFT = 0.0;

    public static double TURRET_SOFT_LIMIT_CCW = 175.0;
    public static double TURRET_SOFT_LIMIT_CW = -210.0;
    public static double TURRET_START_OFFSET_DEG = 0;

    public static double TURRET_TICKS_PER_REV = 32798;
    public static double TICKS_PER_DEGREE = 32798 / 360.0;

    public static double MAX_PHYSICAL_ACCEL = 10000;
    public static double IMPACT_COOLDOWN_MS = 100;

    public static double ALPHA_NORMAL = 0.55;
    public static double ALPHA_IMPACT = 0.05;

    public static double LL_MAX_TRUST_SPEED = 15.0;
    public static double LL_MAX_TRUST_DISTANCE = 100.0;
    public static double LL_MAX_STALENESS_MS = 100.0;

    public static double LL_MULTI_TAG_POS_WEIGHT = 0.8;
    public static double LL_MULTI_TAG_HEADING_WEIGHT = 0.1;
    public static double LL_SINGLE_TAG_BASE_POS_WEIGHT = 0.6;
    public static double LL_SINGLE_TAG_BASE_HEADING_WEIGHT = 0.02;
    public static double LL_MAX_SINGLE_TAG_DIST_M = 3;

    public static double LL_MAX_TELEPORT_INCHES = 15.0;

    public static double TARGET_HITBOX_WIDTH_INCHES = 16;
    public static double MAX_AIM_ANGLE_TOLERANCE = 8.5;

    public static double kV_TURRET = 0.00055;
    public static double kA_TURRET = 0.00043;

    public static double PID_DEADBAND_DEG = 0.3;

    public static double FILTER_RPM_1 = 2800.0;
    public static double FILTER_KA_1 = 0.85;
    public static double FILTER_D_1 = 0.90;

    public static double FILTER_RPM_2 = 3800.0;
    public static double FILTER_KA_2 = 0.60;
    public static double FILTER_D_2 = 0.70;

    public static double FILTER_RPM_3 = 4900.0;
    public static double FILTER_KA_3 = 0.10;
    public static double FILTER_D_3 = 0.25;

    public static double STAGE_THRESHOLD = 30.0;

    public static double kP_far = 0.05;
    public static double kI_far = 0.00;
    public static double kD_far = 0.00005;

    public static double kP_near = 0.03;
    public static double kI_near = 0.00;
    public static double kD_near = 0.0005;
    public static double kS_near = 0.0;
    public static double I_ZONE_near = 3.0;

    public static double STAGE_THRESHOLD_UNWIND = 80.0;

    public static double kP_far_unwind = 1;
    public static double kI_far_unwind = 0.0;
    public static double kD_far_unwind = 0.0;

    public static double kP_near_unwind = 0.025;
    public static double kI_near_unwind = 0.01;
    public static double kD_near_unwind = 0.0015;
    public static double kS_near_unwind = 0.0;
    public static double I_ZONE_near_unwind = 5.0;

    public static double MAX_INTEGRAL_POWER = 0.08;
    public static double ERROR_TOLERANCE = 0.3;

    private long lastLoopTime = 0;
    private double lastRawVxField = 0;
    private double lastRawVyField = 0;
    private double lastSmoothVxField = 0;
    private double lastSmoothVyField = 0;
    private double lastSmoothAxField = 0;
    private double lastSmoothAyField = 0;
    private double lastROmegaDeg = 0;
    private boolean isImpactDetected = false;
    private ElapsedTime impactTimer = new ElapsedTime();
    private double integralSum = 0;
    private double lastError = 0;
    private boolean wasInStage1 = false;
    private String currentStageName = "IDLE";
    private ElapsedTime pidTimer = new ElapsedTime();
    private double currentTurretPower = 0;
    private boolean isLLActiveThisLoop = false;
    private double dynamicToleranceDeg = 2.0;

    private boolean isCurrentlyUnwinding = false;

    private double lastFilteredDerivative = 0.0;
    private double lastFilteredAlphaDeg = 0.0;
    private double currentActiveKaAlpha = 1.0;
    private double currentActiveDAlpha = 1.0;

    private double lastProcessedBotposeX = -9999.0;
    private double lastProcessedBotposeY = -9999.0;

    private double visionOffsetX = 0.0;
    private double visionOffsetY = 0.0;
    private double visionOffsetH_Rad = 0.0;

    private static final int POSE_BUFFER_SIZE = 100;
    private OdoRecord[] poseBuffer = new OdoRecord[POSE_BUFFER_SIZE];
    private int poseBufferIndex = 0;

    private static class OdoRecord {
        long timestampNano;
        double rawX;
        double rawY;
        double rawH_Rad;

        public OdoRecord(long t, double x, double y, double h) {
            this.timestampNano = t;
            this.rawX = x;
            this.rawY = y;
            this.rawH_Rad = h;
        }
    }

    public static class TurretCommand {
        public boolean hasTarget = false;
        public double pidErrorDeg = 0;
        public double targetRpm = 0;
        public double targetPitch = 0;
        public double targetDistance = 0;
        public boolean isUnwinding = false;
        public boolean isAimLocked = false;
    }

    public AutoAimSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        try {
            robotPose = new PinpointPoseProvider(hardwareMap, "odo");
            robotPose.initialize();
        } catch (Exception e) {
            telemetry.addLine("[FATAL] Chassis ODO missing.");
        }
        try {
            ll = hardwareMap.get(Limelight3A.class, "limelight");
            ll.pipelineSwitch(0);
            ll.start();
        } catch (Exception e) {
            telemetry.addLine("[WARN] Limelight Missing.");
        }
        try {
            turretPose = new PinpointPoseProvider(hardwareMap, "turretyaw");
            turretPose.initialize();
        } catch (Exception e) {
            telemetry.addLine("[WARN] Turret hardware missing. Virtual Turret Mode Active.");
        }
        try {
            turretMotor = hardwareMap.get(DcMotorEx.class, "Turret");
            turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } catch (Exception e) {
            telemetry.addLine("[FATAL] Turret Motor missing.");
        }

        for(int i = 0; i < POSE_BUFFER_SIZE; i++) {
            poseBuffer[i] = new OdoRecord(0, 0, 0, 0);
        }

        pidTimer.reset();
    }

    private OdoRecord getHistoricalPose(long targetTimeNano) {
        long minDiff = Long.MAX_VALUE;
        OdoRecord bestRecord = poseBuffer[0];

        for (int i = 0; i < POSE_BUFFER_SIZE; i++) {
            if (poseBuffer[i].timestampNano == 0) continue;
            long diff = Math.abs(poseBuffer[i].timestampNano - targetTimeNano);
            if (diff < minDiff) {
                minDiff = diff;
                bestRecord = poseBuffer[i];
            }
        }
        return bestRecord;
    }

    private double[] getDynamicFilters(double targetRpm) {
        if (targetRpm <= FILTER_RPM_1) {
            return new double[]{FILTER_KA_1, FILTER_D_1};
        }
        if (targetRpm >= FILTER_RPM_3) {
            return new double[]{FILTER_KA_3, FILTER_D_3};
        }

        if (targetRpm < FILTER_RPM_2) {
            double ratio = (targetRpm - FILTER_RPM_1) / (FILTER_RPM_2 - FILTER_RPM_1);
            double ka = FILTER_KA_1 + ratio * (FILTER_KA_2 - FILTER_KA_1);
            double d = FILTER_D_1 + ratio * (FILTER_D_2 - FILTER_D_1);
            return new double[]{ka, d};
        } else {
            double ratio = (targetRpm - FILTER_RPM_2) / (FILTER_RPM_3 - FILTER_RPM_2);
            double ka = FILTER_KA_2 + ratio * (FILTER_KA_3 - FILTER_KA_2);
            double d = FILTER_D_2 + ratio * (FILTER_D_3 - FILTER_D_2);
            return new double[]{ka, d};
        }
    }

    public TurretCommand update(double targetX, double targetY, boolean isShootingMode) {
        TurretCommand command = new TurretCommand();
        if (robotPose == null) return command;

        long currentNanoTime = System.nanoTime();
        if (lastLoopTime == 0) lastLoopTime = currentNanoTime;

        robotPose.update();
        if (turretPose != null) turretPose.update();

        double rawX = -robotPose.getX(DistanceUnit.INCH);
        double rawY = robotPose.getY(DistanceUnit.INCH);
        double rawH_Rad = robotPose.getHeading(AngleUnit.RADIANS);

        double rVx = -robotPose.getXVelocity(DistanceUnit.INCH);
        double rVy = robotPose.getYVelocity(DistanceUnit.INCH);
        double rOmegaRad = robotPose.getHeadingVelocity(AngleUnit.RADIANS);
        double rOmegaDeg = Math.toDegrees(rOmegaRad);
        double speed = Math.hypot(rVx, rVy);

        poseBuffer[poseBufferIndex].timestampNano = currentNanoTime;
        poseBuffer[poseBufferIndex].rawX = rawX;
        poseBuffer[poseBufferIndex].rawY = rawY;
        poseBuffer[poseBufferIndex].rawH_Rad = rawH_Rad;
        poseBufferIndex = (poseBufferIndex + 1) % POSE_BUFFER_SIZE;

        double rX = rawX + visionOffsetX;
        double rY = rawY + visionOffsetY;
        double rH_Rad = AngleUnit.normalizeRadians(rawH_Rad + visionOffsetH_Rad);

        double chassisDistToTarget = Math.hypot(targetX - rX, targetY - rY);

        isLLActiveThisLoop = false;

        if (ll != null && speed < LL_MAX_TRUST_SPEED) {
            LLResult result = ll.getLatestResult();

            if (result != null && result.isValid() && chassisDistToTarget < LL_MAX_TRUST_DISTANCE && result.getStaleness() < LL_MAX_STALENESS_MS) {
                Pose3D botpose = result.getBotpose();
                double llRawX_Meters = botpose.getPosition().x;
                double llRawY_Meters = botpose.getPosition().y;

                if (llRawX_Meters != lastProcessedBotposeX || llRawY_Meters != lastProcessedBotposeY) {
                    lastProcessedBotposeX = llRawX_Meters;
                    lastProcessedBotposeY = llRawY_Meters;

                    double llRawYaw_Rad = botpose.getOrientation().getYaw(AngleUnit.RADIANS);
                    double targetWorldX_Inches = (llRawY_Meters * 39.3701) + FIELD_OFFSET_X;
                    double targetWorldY_Inches = (-llRawX_Meters * 39.3701) + FIELD_OFFSET_Y;
                    double targetWorldH_Rad = AngleUnit.normalizeRadians(llRawYaw_Rad + Math.PI);

                    long totalLatencyNano = (long) ((result.getCaptureLatency() + result.getTargetingLatency()) * 1_000_000.0);
                    long visionTimestampNano = currentNanoTime - totalLatencyNano;

                    OdoRecord histRawOdo = getHistoricalPose(visionTimestampNano);

                    double histRealX = histRawOdo.rawX + visionOffsetX;
                    double histRealY = histRawOdo.rawY + visionOffsetY;
                    double histRealH = AngleUnit.normalizeRadians(histRawOdo.rawH_Rad + visionOffsetH_Rad);

                    double errX = targetWorldX_Inches - histRealX;
                    double errY = targetWorldY_Inches - histRealY;
                    double errH = AngleUnit.normalizeRadians(targetWorldH_Rad - histRealH);

                    int tagCount = result.getFiducialResults().size();
                    double dynamicWeightPos = 0;
                    double dynamicWeightHeading = 0;

                    if (tagCount >= 2) {
                        dynamicWeightPos = LL_MULTI_TAG_POS_WEIGHT;
                        dynamicWeightHeading = LL_MULTI_TAG_HEADING_WEIGHT;
                    } else if (tagCount == 1) {
                        double distZ = botpose.getPosition().z;
                        double trustFactor = Math.max(0.0, 1.0 - (distZ * distZ) / (LL_MAX_SINGLE_TAG_DIST_M * LL_MAX_SINGLE_TAG_DIST_M));
                        dynamicWeightPos = LL_SINGLE_TAG_BASE_POS_WEIGHT * trustFactor;
                        dynamicWeightHeading = LL_SINGLE_TAG_BASE_HEADING_WEIGHT * trustFactor;
                    }

                    double jumpDist = Math.hypot(errX, errY);
                    if (jumpDist < LL_MAX_TELEPORT_INCHES) {
                        visionOffsetX += errX * dynamicWeightPos;
                        visionOffsetY += errY * dynamicWeightPos;
                        visionOffsetH_Rad = AngleUnit.normalizeRadians(visionOffsetH_Rad + errH * dynamicWeightHeading);

                        rX = rawX + visionOffsetX;
                        rY = rawY + visionOffsetY;
                        rH_Rad = AngleUnit.normalizeRadians(rawH_Rad + visionOffsetH_Rad);

                        isLLActiveThisLoop = true;
                    } else {
                        telemetry.addData("[LL Core]", "REJECTED! Teleportation (%.1f in) detected.", jumpDist);
                    }
                }
            }
        }

        double cosH = Math.cos(rH_Rad);
        double sinH = Math.sin(rH_Rad);
        double turretX = rX + (TURRET_OFFSET_FWD * (-sinH)) + (TURRET_OFFSET_LEFT * (-cosH));
        double turretY = rY + (TURRET_OFFSET_FWD * ( cosH)) + (TURRET_OFFSET_LEFT * (-sinH));

        double tanVx = (TURRET_OFFSET_FWD * -cosH * rOmegaRad) + (TURRET_OFFSET_LEFT * sinH * rOmegaRad);
        double tanVy = (TURRET_OFFSET_FWD * -sinH * rOmegaRad) + (TURRET_OFFSET_LEFT * -cosH * rOmegaRad);
        double turretVx = rVx + tanVx;
        double turretVy = rVy + tanVy;

        double dtNano = (currentNanoTime - lastLoopTime);
        double dtSec = dtNano / 1.0E9;
        if(dtSec < 0.001) dtSec = 0.001;

        if (lastLoopTime == 0) {
            lastRawVxField = turretVx;
            lastRawVyField = turretVy;
            lastSmoothVxField = turretVx;
            lastSmoothVyField = turretVy;
            lastROmegaDeg = rOmegaDeg;
        }

        double currentAx = (turretVx - lastRawVxField) / dtSec;
        double currentAy = (turretVy - lastRawVyField) / dtSec;

        double accelMag = Math.hypot(currentAx, currentAy);
        lastRawVxField = turretVx;
        lastRawVyField = turretVy;

        if (accelMag > MAX_PHYSICAL_ACCEL) {
            isImpactDetected = true;
            impactTimer.reset();
        } else if (impactTimer.milliseconds() > IMPACT_COOLDOWN_MS) {
            isImpactDetected = false;
        }

        if (Math.hypot(turretVx, turretVy) < 1.5) {
            turretVx = 0;
            turretVy = 0;
            currentAx = 0;
            currentAy = 0;
            lastSmoothVxField = 0;
            lastSmoothVyField = 0;
            lastSmoothAxField = 0;
            lastSmoothAyField = 0;
        }

        double alpha = isImpactDetected ? ALPHA_IMPACT : ALPHA_NORMAL;
        double smoothVx = lastSmoothVxField * (1.0 - alpha) + turretVx * alpha;
        double smoothVy = lastSmoothVyField * (1.0 - alpha) + turretVy * alpha;

        lastSmoothAxField = lastSmoothAxField * (1.0 - alpha) + currentAx * alpha;
        lastSmoothAyField = lastSmoothAyField * (1.0 - alpha) + currentAy * alpha;

        lastSmoothVxField = smoothVx;
        lastSmoothVyField = smoothVy;
        lastLoopTime = currentNanoTime;

        AimCalculator.AimResult aim = AimCalculator.solveAim(
                turretX, turretY, smoothVx, smoothVy, lastSmoothAxField, lastSmoothAyField, targetX, targetY
        );

        double distanceToTarget = Math.hypot(targetX - turretX, targetY - turretY);
        double feedforwardPower = 0.0;

        dynamicToleranceDeg = Math.toDegrees(Math.atan2(TARGET_HITBOX_WIDTH_INCHES / 2.0, distanceToTarget));
        if(dynamicToleranceDeg > MAX_AIM_ANGLE_TOLERANCE) dynamicToleranceDeg = MAX_AIM_ANGLE_TOLERANCE;
        if(dynamicToleranceDeg < 0.5) dynamicToleranceDeg = 0.5;

        double activeTargetRpm = (aim != null) ? aim.rpm : 2800.0;
        double[] dynamicFilters = getDynamicFilters(activeTargetRpm);
        currentActiveKaAlpha = dynamicFilters[0];
        currentActiveDAlpha = dynamicFilters[1];

        double rawAlphaDeg = (rOmegaDeg - lastROmegaDeg) / dtSec;
        double currentAlphaDeg = (currentActiveKaAlpha * rawAlphaDeg) + ((1.0 - currentActiveKaAlpha) * lastFilteredAlphaDeg);
        lastFilteredAlphaDeg = currentAlphaDeg;
        lastROmegaDeg = rOmegaDeg;

        if (aim != null && turretMotor != null) {
            command.hasTarget = true;
            command.targetRpm = aim.rpm;
            command.targetPitch = aim.pitch;
            command.targetDistance = distanceToTarget;

            double targetAbsHeading = aim.algYaw - 90.0;
            double currentChassisHeading = Math.toDegrees(rH_Rad);

            double encoderDeg = turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;
            double currentTurretRelDegToChassis = encoderDeg + TURRET_START_OFFSET_DEG;
            double desiredRelDegToChassis = targetAbsHeading - currentChassisHeading;
            double shortestPathError = AngleUnit.normalizeDegrees(desiredRelDegToChassis - currentTurretRelDegToChassis);
            double targetEncoderDeg = encoderDeg + shortestPathError;

            if (targetEncoderDeg > TURRET_SOFT_LIMIT_CCW) {
                targetEncoderDeg -= 360.0;
            } else if (targetEncoderDeg < TURRET_SOFT_LIMIT_CW) {
                targetEncoderDeg += 360.0;
            }

            if (targetEncoderDeg > TURRET_SOFT_LIMIT_CCW) targetEncoderDeg = TURRET_SOFT_LIMIT_CCW;
            if (targetEncoderDeg < TURRET_SOFT_LIMIT_CW) targetEncoderDeg = TURRET_SOFT_LIMIT_CW;

            command.pidErrorDeg = targetEncoderDeg - encoderDeg;

            if (Math.abs(command.pidErrorDeg) > 45.0) {
                isCurrentlyUnwinding = true;
            } else if (Math.abs(command.pidErrorDeg) <= 5.0) {
                isCurrentlyUnwinding = false;
            }
            command.isUnwinding = isCurrentlyUnwinding;

            command.isAimLocked = Math.abs(command.pidErrorDeg) <= dynamicToleranceDeg;

            double requiredOmegaFromRotation = -rOmegaDeg;
            double dx = targetX - turretX;
            double dy = targetY - turretY;
            double distSq = dx * dx + dy * dy;
            double tangentialVelocity = (turretVx * dy - turretVy * dx) / Math.sqrt(distSq);
            double requiredOmegaFromTranslation = Math.toDegrees(tangentialVelocity / Math.sqrt(distSq));

            double totalRequiredRelativeOmega = requiredOmegaFromRotation + requiredOmegaFromTranslation;

            double requiredAlphaFromRotation = -currentAlphaDeg;
            double active_kA_TURRET = isShootingMode ? kA_TURRET : 0.0;

            feedforwardPower = (totalRequiredRelativeOmega * kV_TURRET) + (requiredAlphaFromRotation * active_kA_TURRET);
        }

        if (turretMotor != null) {
            if (command.hasTarget) {
                double error = command.pidErrorDeg;
                double absError = Math.abs(error);
                double dt = pidTimer.seconds();
                if (dt == 0) dt = 0.001;

                double effectiveError = error;
                if (absError < PID_DEADBAND_DEG) {
                    effectiveError = 0;
                }
                double effectiveAbsError = Math.abs(effectiveError);

                double rawDerivative = (effectiveError - lastError) / dt;
                double filteredDerivative = (currentActiveDAlpha * rawDerivative) + ((1.0 - currentActiveDAlpha) * lastFilteredDerivative);
                lastFilteredDerivative = filteredDerivative;


                double active_STAGE_THRESHOLD = command.isUnwinding ? STAGE_THRESHOLD_UNWIND : STAGE_THRESHOLD;
                double active_kP_far          = command.isUnwinding ? kP_far_unwind          : kP_far;
                double active_kI_far          = command.isUnwinding ? kI_far_unwind          : kI_far;
                double active_kD_far          = command.isUnwinding ? kD_far_unwind          : kD_far;
                double active_kP_near         = command.isUnwinding ? kP_near_unwind         : kP_near;
                double active_kI_near         = command.isUnwinding ? kI_near_unwind         : kI_near;
                double active_kD_near         = command.isUnwinding ? kD_near_unwind         : kD_near;
                double active_kS_near         = command.isUnwinding ? kS_near_unwind         : kS_near;
                double active_I_ZONE_near     = command.isUnwinding ? I_ZONE_near_unwind     : I_ZONE_near;

                double pidPower = 0.0;

                if (absError <= ERROR_TOLERANCE) {
                    pidPower = 0;
                    integralSum = 0;
                    currentStageName = "【到达目标】";
                    wasInStage1 = false;
                }
                else if (effectiveAbsError > active_STAGE_THRESHOLD) {
                    currentStageName = command.isUnwinding ? "【复位第一段】(远距)" : "【第一段】(远距)";
                    wasInStage1 = true;
                    integralSum += effectiveError * dt;
                    pidPower = (active_kP_far * effectiveError) + (active_kI_far * integralSum) + (active_kD_far * filteredDerivative);
                }
                else {
                    currentStageName = command.isUnwinding ? "【复位第二段】(近距)" : "【第二段】(近距)";
                    if (wasInStage1 || Math.signum(effectiveError) != Math.signum(lastError)) {
                        integralSum = 0;
                        wasInStage1 = false;
                    }
                    if (effectiveAbsError > 0 && effectiveAbsError < active_I_ZONE_near) {
                        integralSum += effectiveError * dt;
                        double maxISum = MAX_INTEGRAL_POWER / (active_kI_near == 0 ? 1 : active_kI_near);
                        integralSum = Math.max(-maxISum, Math.min(maxISum, integralSum));
                    } else if (effectiveAbsError == 0) {
                    } else {
                        integralSum = 0;
                    }
                    double frictionFF = (effectiveAbsError == 0) ? 0 : (Math.signum(effectiveError) * active_kS_near);
                    pidPower = (active_kP_near * effectiveError) + (active_kI_near * integralSum) + (active_kD_near * filteredDerivative) + frictionFF;
                }

                double finalPower = pidPower + feedforwardPower;
                finalPower = Math.max(-1.0, Math.min(1.0, finalPower));

                turretMotor.setPower(finalPower);
                currentTurretPower = finalPower;
                lastError = effectiveError;
                pidTimer.reset();

            } else {
                turretMotor.setPower(0);
                currentTurretPower = 0;
                integralSum = 0;
                lastError = 0;
                wasInStage1 = false;
                currentStageName = "LOST/IDLE";
                pidTimer.reset();
            }
        }

        printTelemetry(aim, command, rX, rY, targetX, targetY, feedforwardPower, isShootingMode);

        return command;
    }

    private void printTelemetry(AimCalculator.AimResult aim, TurretCommand command, double rX, double rY, double targetX, double targetY, double ffPower, boolean isShootingMode) {
        telemetry.addData("AutoAim Status", isImpactDetected ? "[! IMPACT !]" : "[ OK ]");
        telemetry.addData("Chassis Pos", "X:%.1f  Y:%.1f", rX, rY);
        telemetry.addData("Vision Filter", isLLActiveThisLoop ? "ACTIVE (Time-Machine Sync)" : "DISABLED (Speed/Dist)");

        if(aim != null && command.hasTarget) {
            telemetry.addData("Target Locked", "WorldX:%.0f WorldY:%.0f | Dist: %.1f in", targetX, targetY, command.targetDistance);
            telemetry.addData("Aim Command", "RPM: %.0f | Pitch: %.2f", aim.rpm, aim.pitch);

            telemetry.addData("Dynamic Filters", "kA_Alpha: %.2f | D_Alpha: %.2f", currentActiveKaAlpha, currentActiveDAlpha);
            telemetry.addData("kA (Alpha FF) Status", isShootingMode ? "ACTIVE" : "DISABLED (Idle)");

            double encDeg = turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;

            telemetry.addData("Turret Control", "%s Pow: %.2f (FF: %.2f) | Err: %.1f°",
                    currentStageName, currentTurretPower, ffPower, command.pidErrorDeg);
            telemetry.addData("Dynamic Tolerance", "±%.2f°", dynamicToleranceDeg);

            telemetry.addData("Turret Angle", "Encoder: %.1f° | RelToFront: %.1f°",
                    encDeg, AngleUnit.normalizeDegrees(encDeg + TURRET_START_OFFSET_DEG));
        } else {
            telemetry.addLine("[Target] LOST or TOO CLOSE");
        }
    }

    public void setInitialPose(Pose2D pose) {
        if (robotPose != null) {
            robotPose.setPose(pose);
            robotPose.update();

            visionOffsetX = 0;
            visionOffsetY = 0;
            visionOffsetH_Rad = 0;

            for(int i = 0; i < POSE_BUFFER_SIZE; i++) {
                if (poseBuffer[i] != null) {
                    poseBuffer[i].timestampNano = 0;
                }
            }

            telemetry.addData("[System]", "Odometry & Offsets Fully Reset");
        }
    }
}