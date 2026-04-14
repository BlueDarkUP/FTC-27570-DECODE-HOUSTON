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

    // --------------------------------------------------------
    // 【DASHBOARD 参数分组区域】
    // 将变量放入 public static class 中即可在 Dashboard 中生成折叠文件夹
    // --------------------------------------------------------

    public static class Offsets {
        public static double FIELD_OFFSET_X = 72.0;
        public static double FIELD_OFFSET_Y = 72.0;
        public static double TURRET_OFFSET_FWD = -1.22;
        public static double TURRET_OFFSET_LEFT = 0.0;
        public static double TURRET_START_OFFSET_DEG = 0;
    }

    public static class HWLimit {
        public static double TURRET_SOFT_LIMIT_CCW = 175.0;
        public static double TURRET_SOFT_LIMIT_CW = -210.0;
        public static double TURRET_TICKS_PER_REV = 32798;
        public static double TICKS_PER_DEGREE = 32798 / 360.0;

        public static double MAX_PHYSICAL_ACCEL = 10000;
        public static double IMPACT_COOLDOWN_MS = 100;
        public static double ALPHA_NORMAL = 0.55;
        public static double ALPHA_IMPACT = 0.05;
    }

    public static class LLConfig {
        public static double LL_MAX_TRUST_SPEED = 15.0;
        public static double LL_MAX_TRUST_DISTANCE = 100.0;
        public static double LL_MAX_STALENESS_MS = 100.0;

        public static double LL_MULTI_TAG_POS_WEIGHT = 0.8;
        public static double LL_MULTI_TAG_HEADING_WEIGHT = 0.1;
        public static double LL_SINGLE_TAG_BASE_POS_WEIGHT = 0.6;
        public static double LL_SINGLE_TAG_BASE_HEADING_WEIGHT = 0.02;
        public static double LL_MAX_SINGLE_TAG_DIST_M = 3;

        public static double LL_MAX_TELEPORT_INCHES = 15.0;
    }

    public static class AimConfig {
        public static double TARGET_HITBOX_WIDTH_INCHES = 16;
        public static double MAX_AIM_ANGLE_TOLERANCE = 8.5;
        public static double TARGET_SLEW_RATE_DEG_PER_SEC = 600.0;

        public static double kV_TURRET = 0.00055;
        public static double kA_TURRET = 0.00043;
        public static double PID_DEADBAND_DEG = 0.3;
    }

    public static class FilterMap {
        public static double FILTER_RPM_1 = 2800.0;
        public static double FILTER_KA_1 = 0.85;
        public static double FILTER_D_1 = 0.90;

        public static double FILTER_RPM_2 = 3800.0;
        public static double FILTER_KA_2 = 0.60;
        public static double FILTER_D_2 = 0.70;

        public static double FILTER_RPM_3 = 4900.0;
        public static double FILTER_KA_3 = 0.10;
        public static double FILTER_D_3 = 0.15;
    }

    public static class AutoPID {
        public static double STAGE_THRESHOLD = 30.0;
        public static double ERROR_TOLERANCE = 0.3;
        public static double MAX_INTEGRAL_POWER = 0.08;

        public static double kP_far = 0.05;
        public static double kI_far = 0.00;
        public static double kD_far = 0.00005;

        public static double kP_near = 0.03;
        public static double kI_near = 0.00;
        public static double kD_near = 0.0005;
        public static double kS_near = 0.0;
        public static double I_ZONE_near = 3.0;
    }

    public static class UnwindPID {
        public static double STAGE_THRESHOLD_UNWIND = 80.0;

        public static double kP_far_unwind = 1;
        public static double kI_far_unwind = 0.0;
        public static double kD_far_unwind = 0.0;

        public static double kP_near_unwind = 0.025;
        public static double kI_near_unwind = 0.01;
        public static double kD_near_unwind = 0.0015;
        public static double kS_near_unwind = 0.0;
        public static double I_ZONE_near_unwind = 5.0;
    }

    public static class ManualPID {
        public static double MANUAL_STAGE_THRESHOLD = 14.0;
        public static double MANUAL_MAX_INTEGRAL_POWER = 0.08;
        public static double MANUAL_ERROR_TOLERANCE = 0.3;

        public static double kP_far_manual = 0.05;
        public static double kI_far_manual = 0.00;
        public static double kD_far_manual = 0.0003;

        public static double kP_near_manual = 0.00002;
        public static double kI_near_manual = 0.0001;
        public static double kD_near_manual = 0.0015;
        public static double kS_near_manual = 0.28;
        public static double I_ZONE_near_manual = 3.0;
    }

    // --------------------------------------------------------
    // 【系统内部变量】
    // --------------------------------------------------------

    private PinpointPoseProvider robotPose;
    private PinpointPoseProvider turretPose;
    private Limelight3A ll;
    private Telemetry telemetry;
    private DcMotorEx turretMotor;

    private boolean isAutonomousMode = false;
    private double extX, extY, extH, extVx, extVy, extOmega;

    private double profiledTargetDeg = Double.NaN;
    private double lastEncoderDegForDeriv = 0.0;

    private long lastLoopTime = 0;
    private double lastRawVxField = 0, lastRawVyField = 0;
    private double lastSmoothVxField = 0, lastSmoothVyField = 0;
    private double lastSmoothAxField = 0, lastSmoothAyField = 0;
    private double lastROmegaDeg = 0;
    private boolean isImpactDetected = false;
    private ElapsedTime impactTimer = new ElapsedTime();

    private double integralSum = 0;

    private double manualIntegralSum = 0;
    private boolean isManualBrakeLocked = false;

    private String currentStageName = "IDLE";
    private ElapsedTime pidTimer = new ElapsedTime();
    private double currentTurretPower = 0;
    private boolean isLLActiveThisLoop = false;
    private double dynamicToleranceDeg = 2.0;

    private boolean isCurrentlyUnwinding = false;
    private double lastFilteredTurretVel = 0.0;
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
            this.timestampNano = t; this.rawX = x; this.rawY = y; this.rawH_Rad = h;
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
        this(hardwareMap, telemetry, false);
    }

    public AutoAimSubsystem(HardwareMap hardwareMap, Telemetry telemetry, boolean isAutonomousMode) {
        this.telemetry = telemetry;
        this.isAutonomousMode = isAutonomousMode;

        if (!isAutonomousMode) {
            try {
                robotPose = new PinpointPoseProvider(hardwareMap, "odo");
                robotPose.initialize();
            } catch (Exception e) {
                telemetry.addLine("[FATAL] Chassis ODO missing.");
            }
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
            telemetry.addLine("[WARN] Turret hardware missing.");
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

    public void updateExternalOdometry(double x, double y, double h, double vx, double vy, double omega) {
        this.extX = x; this.extY = y; this.extH = h;
        this.extVx = vx; this.extVy = vy; this.extOmega = omega;
    }

    private OdoRecord getHistoricalPose(long targetTimeNano) {
        long minDiff = Long.MAX_VALUE;
        OdoRecord bestRecord = poseBuffer[0];
        for (int i = 0; i < POSE_BUFFER_SIZE; i++) {
            if (poseBuffer[i].timestampNano == 0) continue;
            long diff = Math.abs(poseBuffer[i].timestampNano - targetTimeNano);
            if (diff < minDiff) { minDiff = diff; bestRecord = poseBuffer[i]; }
        }
        return bestRecord;
    }

    private double[] getDynamicFilters(double targetRpm) {
        if (targetRpm <= FilterMap.FILTER_RPM_1) return new double[]{FilterMap.FILTER_KA_1, FilterMap.FILTER_D_1};
        if (targetRpm >= FilterMap.FILTER_RPM_3) return new double[]{FilterMap.FILTER_KA_3, FilterMap.FILTER_D_3};
        if (targetRpm < FilterMap.FILTER_RPM_2) {
            double ratio = (targetRpm - FilterMap.FILTER_RPM_1) / (FilterMap.FILTER_RPM_2 - FilterMap.FILTER_RPM_1);
            return new double[]{FilterMap.FILTER_KA_1 + ratio * (FilterMap.FILTER_KA_2 - FilterMap.FILTER_KA_1), FilterMap.FILTER_D_1 + ratio * (FilterMap.FILTER_D_2 - FilterMap.FILTER_D_1)};
        } else {
            double ratio = (targetRpm - FilterMap.FILTER_RPM_2) / (FilterMap.FILTER_RPM_3 - FilterMap.FILTER_RPM_2);
            return new double[]{FilterMap.FILTER_KA_2 + ratio * (FilterMap.FILTER_KA_3 - FilterMap.FILTER_KA_2), FilterMap.FILTER_D_2 + ratio * (FilterMap.FILTER_D_3 - FilterMap.FILTER_D_2)};
        }
    }

    public TurretCommand update(double targetX, double targetY, boolean isShootingMode, boolean isManualMode, double manualDistance, boolean emergencyBrake) {
        TurretCommand command = new TurretCommand();

        long currentNanoTime = System.nanoTime();
        if (lastLoopTime == 0) lastLoopTime = currentNanoTime;

        if (turretPose != null) turretPose.update();

        double rawX, rawY, rawH_Rad, rVx, rVy, rOmegaRad;
        if (isAutonomousMode) {
            rawX = extX; rawY = extY; rawH_Rad = extH; rVx = extVx; rVy = extVy; rOmegaRad = extOmega;
        } else {
            if (robotPose == null) return command;
            robotPose.update();
            rawX = -robotPose.getX(DistanceUnit.INCH);
            rawY = robotPose.getY(DistanceUnit.INCH);
            rawH_Rad = robotPose.getHeading(AngleUnit.RADIANS);
            rVx = -robotPose.getXVelocity(DistanceUnit.INCH);
            rVy = robotPose.getYVelocity(DistanceUnit.INCH);
            rOmegaRad = robotPose.getHeadingVelocity(AngleUnit.RADIANS);
        }

        double speed = Math.hypot(rVx, rVy);
        double rOmegaDeg = Math.toDegrees(rOmegaRad);

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
        if (ll != null && speed < LLConfig.LL_MAX_TRUST_SPEED) {
            LLResult result = ll.getLatestResult();
            if (result != null && result.isValid() && chassisDistToTarget < LLConfig.LL_MAX_TRUST_DISTANCE && result.getStaleness() < LLConfig.LL_MAX_STALENESS_MS) {
                Pose3D botpose = result.getBotpose();
                double llRawX_Meters = botpose.getPosition().x;
                double llRawY_Meters = botpose.getPosition().y;
                if (llRawX_Meters != lastProcessedBotposeX || llRawY_Meters != lastProcessedBotposeY) {
                    lastProcessedBotposeX = llRawX_Meters; lastProcessedBotposeY = llRawY_Meters;
                    double llRawYaw_Rad = botpose.getOrientation().getYaw(AngleUnit.RADIANS);
                    double targetWorldX_Inches = (llRawY_Meters * 39.3701) + Offsets.FIELD_OFFSET_X;
                    double targetWorldY_Inches = (-llRawX_Meters * 39.3701) + Offsets.FIELD_OFFSET_Y;
                    double targetWorldH_Rad = AngleUnit.normalizeRadians(llRawYaw_Rad + Math.PI);
                    long totalLatencyNano = (long) ((result.getCaptureLatency() + result.getTargetingLatency()) * 1_000_000.0);
                    OdoRecord histRawOdo = getHistoricalPose(currentNanoTime - totalLatencyNano);
                    double histRealX = histRawOdo.rawX + visionOffsetX;
                    double histRealY = histRawOdo.rawY + visionOffsetY;
                    double histRealH = AngleUnit.normalizeRadians(histRawOdo.rawH_Rad + visionOffsetH_Rad);
                    double errX = targetWorldX_Inches - histRealX;
                    double errY = targetWorldY_Inches - histRealY;
                    double errH = AngleUnit.normalizeRadians(targetWorldH_Rad - histRealH);
                    int tagCount = result.getFiducialResults().size();
                    double dynamicWeightPos = (tagCount >= 2) ? LLConfig.LL_MULTI_TAG_POS_WEIGHT : LLConfig.LL_SINGLE_TAG_BASE_POS_WEIGHT * Math.max(0.0, 1.0 - Math.pow(botpose.getPosition().z/LLConfig.LL_MAX_SINGLE_TAG_DIST_M, 2));
                    double dynamicWeightHeading = (tagCount >= 2) ? LLConfig.LL_MULTI_TAG_HEADING_WEIGHT : LLConfig.LL_SINGLE_TAG_BASE_HEADING_WEIGHT * Math.max(0.0, 1.0 - Math.pow(botpose.getPosition().z/LLConfig.LL_MAX_SINGLE_TAG_DIST_M, 2));
                    double jumpDist = Math.hypot(errX, errY);
                    if (jumpDist < LLConfig.LL_MAX_TELEPORT_INCHES) {
                        visionOffsetX += errX * dynamicWeightPos;
                        visionOffsetY += errY * dynamicWeightPos;
                        visionOffsetH_Rad = AngleUnit.normalizeRadians(visionOffsetH_Rad + errH * dynamicWeightHeading);
                        rX = rawX + visionOffsetX; rY = rawY + visionOffsetY; rH_Rad = AngleUnit.normalizeRadians(rawH_Rad + visionOffsetH_Rad);
                        isLLActiveThisLoop = true;
                    }
                }
            }
        }

        double cosH = Math.cos(rH_Rad);
        double sinH = Math.sin(rH_Rad);
        double turretX = rX + (Offsets.TURRET_OFFSET_FWD * (-sinH)) + (Offsets.TURRET_OFFSET_LEFT * (-cosH));
        double turretY = rY + (Offsets.TURRET_OFFSET_FWD * ( cosH)) + (Offsets.TURRET_OFFSET_LEFT * (-sinH));
        double dtNano = (currentNanoTime - lastLoopTime);
        double dtSec = Math.max(dtNano / 1.0E9, 0.001);

        double tanVx = (Offsets.TURRET_OFFSET_FWD * -cosH * rOmegaRad) + (Offsets.TURRET_OFFSET_LEFT * sinH * rOmegaRad);
        double tanVy = (Offsets.TURRET_OFFSET_FWD * -sinH * rOmegaRad) + (Offsets.TURRET_OFFSET_LEFT * -cosH * rOmegaRad);
        double turretVx = rVx + tanVx;
        double turretVy = rVy + tanVy;

        if (lastLoopTime == 0) { lastRawVxField = turretVx; lastRawVyField = turretVy; lastSmoothVxField = turretVx; lastSmoothVyField = turretVy; lastROmegaDeg = rOmegaDeg; }
        double currentAx = (turretVx - lastRawVxField) / dtSec;
        double currentAy = (turretVy - lastRawVyField) / dtSec;
        if (Math.hypot(currentAx, currentAy) > HWLimit.MAX_PHYSICAL_ACCEL) { isImpactDetected = true; impactTimer.reset(); }
        else if (impactTimer.milliseconds() > HWLimit.IMPACT_COOLDOWN_MS) isImpactDetected = false;

        lastRawVxField = turretVx; lastRawVyField = turretVy;
        double alpha = isImpactDetected ? HWLimit.ALPHA_IMPACT : HWLimit.ALPHA_NORMAL;
        double smoothVx = lastSmoothVxField * (1.0 - alpha) + turretVx * alpha;
        double smoothVy = lastSmoothVyField * (1.0 - alpha) + turretVy * alpha;
        lastSmoothAxField = lastSmoothAxField * (1.0 - alpha) + currentAx * alpha;
        lastSmoothAyField = lastSmoothAyField * (1.0 - alpha) + currentAy * alpha;
        lastSmoothVxField = smoothVx; lastSmoothVyField = smoothVy;
        lastLoopTime = currentNanoTime;

        AimCalculator.AimResult aim = null;
        double feedforwardPower = 0.0;
        double encoderDeg = turretMotor != null ? (turretMotor.getCurrentPosition() / HWLimit.TICKS_PER_DEGREE) : 0;
        double currentAngleRelToChassis = encoderDeg + Offsets.TURRET_START_OFFSET_DEG;

        double currentTurretVelDeg = (encoderDeg - lastEncoderDegForDeriv) / dtSec;
        lastEncoderDegForDeriv = encoderDeg;

        if (emergencyBrake) {
            if (turretMotor != null) turretMotor.setPower(0);
            integralSum = 0;
            manualIntegralSum = 0;
            isManualBrakeLocked = false;
            currentStageName = "【紧急刹车锁死】";
            currentTurretPower = 0;
            command.hasTarget = false;
            profiledTargetDeg = Double.NaN;
            return command;

        } else if (isManualMode) {
            double vTargetX = turretX + manualDistance * (-sinH);
            double vTargetY = turretY + manualDistance * (cosH);
            aim = AimCalculator.solveAim(turretX, turretY, 0, 0, 0, 0, vTargetX, vTargetY);

            command.hasTarget = true;
            command.targetRpm = (aim != null) ? aim.rpm : 3000.0;
            command.targetPitch = (aim != null) ? aim.pitch : 0.5;
            command.targetDistance = manualDistance;
            command.isAimLocked = true;
            command.isUnwinding = false;

            integralSum = 0;
            profiledTargetDeg = Double.NaN;

            double error = AngleUnit.normalizeDegrees(0.0 - currentAngleRelToChassis);
            command.pidErrorDeg = error;
            double absError = Math.abs(error);
            double power = 0.0;

            if (isManualBrakeLocked) {
                power = 0;
                currentStageName = "【手动到达锁定】(BRAKE)";
            } else {
                if (absError <= ManualPID.MANUAL_ERROR_TOLERANCE) {
                    isManualBrakeLocked = true;
                    power = 0;
                    manualIntegralSum = 0;
                    currentStageName = "【手动到达锁定】(BRAKE)";
                } else {
                    currentStageName = "【手动平滑控制】";

                    double ratio = Math.min(1.0, absError / ManualPID.MANUAL_STAGE_THRESHOLD);
                    double currentP = ManualPID.kP_near_manual + ratio * (ManualPID.kP_far_manual - ManualPID.kP_near_manual);
                    double currentI = ManualPID.kI_near_manual + ratio * (ManualPID.kI_far_manual - ManualPID.kI_near_manual);
                    double currentD = ManualPID.kD_near_manual + ratio * (ManualPID.kD_far_manual - ManualPID.kD_near_manual);

                    if (absError < ManualPID.I_ZONE_near_manual) {
                        manualIntegralSum += error * dtSec;
                        double maxISum = ManualPID.MANUAL_MAX_INTEGRAL_POWER / (currentI == 0 ? 1 : currentI);
                        manualIntegralSum = Math.max(-maxISum, Math.min(maxISum, manualIntegralSum));
                    } else {
                        manualIntegralSum = 0;
                    }

                    double derivativeTerm = -currentD * currentTurretVelDeg;
                    double pidPower = (currentP * error) + (currentI * manualIntegralSum) + derivativeTerm;
                    double feedforward = (absError < ManualPID.MANUAL_STAGE_THRESHOLD) ? (Math.signum(error) * ManualPID.kS_near_manual) : 0;
                    power = pidPower + feedforward;
                }
            }

            power = Math.max(-1.0, Math.min(1.0, power));
            if (turretMotor != null) turretMotor.setPower(power);
            currentTurretPower = power;

        } else {
            isManualBrakeLocked = false;
            manualIntegralSum = 0;

            aim = AimCalculator.solveAim(turretX, turretY, smoothVx, smoothVy, lastSmoothAxField, lastSmoothAyField, targetX, targetY);
            double distanceToTarget = Math.hypot(targetX - turretX, targetY - turretY);

            dynamicToleranceDeg = Math.max(0.5, Math.min(AimConfig.MAX_AIM_ANGLE_TOLERANCE, Math.toDegrees(Math.atan2(AimConfig.TARGET_HITBOX_WIDTH_INCHES / 2.0, distanceToTarget))));

            double activeTargetRpm = (aim != null) ? aim.rpm : 2800.0;
            double[] filters = getDynamicFilters(activeTargetRpm);
            currentActiveKaAlpha = filters[0]; currentActiveDAlpha = filters[1];

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
                double desiredRelDegToChassis = targetAbsHeading - currentChassisHeading;
                double shortestPathError = AngleUnit.normalizeDegrees(desiredRelDegToChassis - currentAngleRelToChassis);
                double rawTargetEncoderDeg = encoderDeg + shortestPathError;

                if (rawTargetEncoderDeg > HWLimit.TURRET_SOFT_LIMIT_CCW) rawTargetEncoderDeg -= 360.0;
                else if (rawTargetEncoderDeg < HWLimit.TURRET_SOFT_LIMIT_CW) rawTargetEncoderDeg += 360.0;
                rawTargetEncoderDeg = Math.max(HWLimit.TURRET_SOFT_LIMIT_CW, Math.min(HWLimit.TURRET_SOFT_LIMIT_CCW, rawTargetEncoderDeg));

                command.pidErrorDeg = rawTargetEncoderDeg - encoderDeg;

                isCurrentlyUnwinding = Math.abs(command.pidErrorDeg) > 45.0 ? true : (Math.abs(command.pidErrorDeg) <= 5.0 ? false : isCurrentlyUnwinding);
                command.isUnwinding = isCurrentlyUnwinding;
                command.isAimLocked = Math.abs(command.pidErrorDeg) <= dynamicToleranceDeg;

                if (command.isUnwinding || Double.isNaN(profiledTargetDeg)) {
                    profiledTargetDeg = rawTargetEncoderDeg;
                } else {
                    double maxStep = AimConfig.TARGET_SLEW_RATE_DEG_PER_SEC * dtSec;
                    if (Math.abs(rawTargetEncoderDeg - profiledTargetDeg) <= maxStep) {
                        profiledTargetDeg = rawTargetEncoderDeg;
                    } else {
                        profiledTargetDeg += Math.signum(rawTargetEncoderDeg - profiledTargetDeg) * maxStep;
                    }
                }

                double activePidError = profiledTargetDeg - encoderDeg;

                double dx = targetX - turretX, dy = targetY - turretY, distSq = dx * dx + dy * dy;
                double tangentialVelocity = (turretVx * dy - turretVy * dx) / Math.sqrt(distSq);
                double reqOmegaTrans = Math.toDegrees(tangentialVelocity / Math.sqrt(distSq));
                feedforwardPower = ((-rOmegaDeg + reqOmegaTrans) * AimConfig.kV_TURRET) + (-currentAlphaDeg * (isShootingMode ? AimConfig.kA_TURRET : 0.0));

                double effectiveError = Math.abs(activePidError) < AimConfig.PID_DEADBAND_DEG ? 0 : activePidError;
                double effectiveAbsError = Math.abs(effectiveError);

                double aSTAGE = command.isUnwinding ? UnwindPID.STAGE_THRESHOLD_UNWIND : AutoPID.STAGE_THRESHOLD;
                double aP_f = command.isUnwinding ? UnwindPID.kP_far_unwind : AutoPID.kP_far, aI_f = command.isUnwinding ? UnwindPID.kI_far_unwind : AutoPID.kI_far, aD_f = command.isUnwinding ? UnwindPID.kD_far_unwind : AutoPID.kD_far;
                double aP_n = command.isUnwinding ? UnwindPID.kP_near_unwind : AutoPID.kP_near, aI_n = command.isUnwinding ? UnwindPID.kI_near_unwind : AutoPID.kI_near, aD_n = command.isUnwinding ? UnwindPID.kD_near_unwind : AutoPID.kD_near;
                double aS_n = command.isUnwinding ? UnwindPID.kS_near_unwind : AutoPID.kS_near, aI_zone = command.isUnwinding ? UnwindPID.I_ZONE_near_unwind : AutoPID.I_ZONE_near;

                double pidPower = 0;
                if (Math.abs(command.pidErrorDeg) <= AutoPID.ERROR_TOLERANCE) {
                    pidPower = 0; integralSum = 0; currentStageName = "【到达目标】";
                } else {
                    currentStageName = command.isUnwinding ? "【复位平滑追踪】" : "【自动平滑追踪】";

                    double blendRatio = Math.min(1.0, effectiveAbsError / aSTAGE);
                    double currentP = aP_n + blendRatio * (aP_f - aP_n);
                    double currentI = aI_n + blendRatio * (aI_f - aI_n);
                    double currentD = aD_n + blendRatio * (aD_f - aD_n);

                    if (effectiveAbsError > 0 && effectiveAbsError < aI_zone) {
                        integralSum += effectiveError * dtSec;
                        double maxISum = AutoPID.MAX_INTEGRAL_POWER / (currentI == 0 ? 1 : currentI);
                        integralSum = Math.max(-maxISum, Math.min(maxISum, integralSum));
                    } else {
                        integralSum = 0;
                    }

                    double rawDerivativeTerm = -currentD * currentTurretVelDeg;
                    double filteredDeriv = (currentActiveDAlpha * rawDerivativeTerm) + ((1.0 - currentActiveDAlpha) * lastFilteredTurretVel);
                    lastFilteredTurretVel = filteredDeriv;

                    pidPower = (currentP * effectiveError) + (currentI * integralSum) + filteredDeriv + ((effectiveAbsError == 0) ? 0 : (Math.signum(effectiveError) * aS_n));
                }

                double finalPower = Math.max(-1.0, Math.min(1.0, pidPower + feedforwardPower));
                turretMotor.setPower(finalPower);
                currentTurretPower = finalPower;
                pidTimer.reset();
            } else {
                if (turretMotor != null) turretMotor.setPower(0);
                currentTurretPower = 0; integralSum = 0;
                profiledTargetDeg = Double.NaN;
                currentStageName = "LOST/IDLE"; pidTimer.reset();
            }
        }

        printTelemetry(aim, command, rX, rY, isManualMode, emergencyBrake);
        return command;
    }

    private void printTelemetry(AimCalculator.AimResult aim, TurretCommand command, double rX, double rY, boolean isManual, boolean emergencyBrake) {
        if (emergencyBrake) {
            telemetry.addData(">> [⚠️ WARNING ⚠️]", "EMERGENCY BRAKE ACTIVE! All Motors Stopped!");
        } else if (isManual) {
            telemetry.addData(">> [⚙️ MODE]", "MANUAL OVERRIDE ACTIVE (Target: %.1f in)", command.targetDistance);
        } else {
            telemetry.addData(">> [🟢 MODE]", "AUTO ODOMETRY TRACKING ACTIVE");
        }
        telemetry.addData("Chassis Pos", "X:%.1f  Y:%.1f", rX, rY);
        if (aim != null && command.hasTarget) {
            telemetry.addData("Aim Target", "RPM: %.0f | Pitch: %.2f", aim.rpm, aim.pitch);
            telemetry.addData("Turret Control", "%s Pow: %.2f | Err: %.1f°", currentStageName, currentTurretPower, command.pidErrorDeg);
        }
    }

    public void setInitialPose(Pose2D pose) {
        if (!isAutonomousMode && robotPose != null) { robotPose.setPose(pose); robotPose.update(); }
        visionOffsetX = 0; visionOffsetY = 0; visionOffsetH_Rad = 0;
        for(int i = 0; i < POSE_BUFFER_SIZE; i++) if (poseBuffer[i] != null) poseBuffer[i].timestampNano = 0;
    }
}