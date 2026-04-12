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

    // ==========================================================
    // === 【Dashboard 调参区】基础物理结构与坐标映射 ===
    // ==========================================================
    public static double FIELD_OFFSET_X = 72.0;
    public static double FIELD_OFFSET_Y = 72.0;
    public static double TURRET_OFFSET_FWD = -2.5;
    public static double TURRET_OFFSET_LEFT = 0.0;

    public static double TURRET_SOFT_LIMIT_CCW = 175.0;
    public static double TURRET_SOFT_LIMIT_CW = -210.0;
    public static double TURRET_START_OFFSET_DEG = 0;

    // 这两个一般是固定的硬件参数，但也暴露出来方便确认
    public static double TURRET_TICKS_PER_REV = 32798;
    public static double TICKS_PER_DEGREE = 32798 / 360.0;

    // ==========================================================
    // === 【Dashboard 调参区】视觉与撞击过滤系统 ===
    // ==========================================================
    public static double MAX_PHYSICAL_ACCEL = 10000;
    public static double IMPACT_COOLDOWN_MS = 100;
    public static double ALPHA_NORMAL = 0.80;
    public static double ALPHA_IMPACT = 0.05;

    public static double LL_BASE_FILTER_WEIGHT = 0.7;
    public static double LL_MAX_TRUST_DISTANCE = 120.0;

    // 目标的有效击打宽度(英寸)，用于动态计算允许的角度误差
    public static double TARGET_HITBOX_WIDTH_INCHES = 8.0;
    // 保底最大角度误差，防止离得太近时容差过大
    public static double MAX_AIM_ANGLE_TOLERANCE = 3.0;

    // ==========================================================
    // === 【Dashboard 调参区】动力学前馈控制 (Feed-Forward) ===
    // ==========================================================
    // 云台电机的速度前馈系数：让云台转动 1度/秒 需要多少电机 Power？
    public static double kV_TURRET = 0.00058;
    // 云台电机的加速度前馈系数：克服云台惯性
    public static double kA_TURRET = 0.00002;

    // ==========================================================
    // === 【Dashboard 调参区】双段 PID 控制参数 ===
    // ==========================================================
    // 切换阈值：误差大于此值用第一段，小于此值用第二段
    public static double STAGE_THRESHOLD = 14.0;

    public static double kP_far = 0.05;
    public static double kI_far = 0.00;
    public static double kD_far = 0.0003;

    public static double kP_near = 0.03;
    public static double kI_near = 0.00005;
    public static double kD_near = 0.000001;

    public static double kS_near = 0.05;
    public static double I_ZONE_near = 3.0;
    public static double MAX_INTEGRAL_POWER = 0.08;
    public static double ERROR_TOLERANCE = 0.3;


    // ==========================================================
    // === 内部状态变量 (不需要在 Dashboard 调试) ===
    // ==========================================================
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
        pidTimer.reset();
    }

    public TurretCommand update(double targetX, double targetY) {
        TurretCommand command = new TurretCommand();
        if (robotPose == null) return command;

        if (lastLoopTime == 0) lastLoopTime = System.nanoTime();

        robotPose.update();
        if (turretPose != null) turretPose.update();

        double rX = -robotPose.getX(DistanceUnit.INCH);
        double rY = robotPose.getY(DistanceUnit.INCH);
        double rH_Rad = robotPose.getHeading(AngleUnit.RADIANS);
        double rVx = -robotPose.getXVelocity(DistanceUnit.INCH);
        double rVy = robotPose.getYVelocity(DistanceUnit.INCH);

        double rOmegaRad = robotPose.getHeadingVelocity(AngleUnit.RADIANS);
        double rOmegaDeg = Math.toDegrees(rOmegaRad);

        double speed = Math.hypot(rVx, rVy);
        double chassisDistToTarget = Math.hypot(targetX - rX, targetY - rY);

        double dynamicLLWeight = LL_BASE_FILTER_WEIGHT;
        if (speed > 5.0) {
            dynamicLLWeight = LL_BASE_FILTER_WEIGHT * (5.0 / speed);
        }

        isLLActiveThisLoop = false;
        if (ll != null) {
            LLResult result = ll.getLatestResult();
            if (result != null && result.isValid() && chassisDistToTarget < LL_MAX_TRUST_DISTANCE) {
                Pose3D botpose = result.getBotpose();
                double llRawX_Meters = botpose.getPosition().x;
                double llRawY_Meters = botpose.getPosition().y;
                double llRawYaw_Rad = botpose.getOrientation().getYaw(AngleUnit.RADIANS);

                double targetWorldX_Inches = (llRawY_Meters * 39.3701) + FIELD_OFFSET_X;
                double targetWorldY_Inches = (-llRawX_Meters * 39.3701) + FIELD_OFFSET_Y;
                double mappedHeading_Rad = AngleUnit.normalizeRadians(llRawYaw_Rad + Math.PI);

                double latency = (result.getCaptureLatency() + result.getTargetingLatency()) / 1000.0;

                Pose2D targetLLPose = new Pose2D(DistanceUnit.INCH,
                        targetWorldX_Inches + (rVx * latency),
                        targetWorldY_Inches + (rVy * latency),
                        AngleUnit.RADIANS, mappedHeading_Rad);

                double currentOdoX = -robotPose.getX(DistanceUnit.INCH);
                double currentOdoY = robotPose.getY(DistanceUnit.INCH);
                double currentOdoH = robotPose.getHeading(AngleUnit.RADIANS);

                double newX = currentOdoX + (targetLLPose.getX(DistanceUnit.INCH) - currentOdoX) * dynamicLLWeight;
                double newY = currentOdoY + (targetLLPose.getY(DistanceUnit.INCH) - currentOdoY) * dynamicLLWeight;

                double angleDiff = AngleUnit.normalizeRadians(targetLLPose.getHeading(AngleUnit.RADIANS) - currentOdoH);
                double newH = AngleUnit.normalizeRadians(currentOdoH + (angleDiff * dynamicLLWeight));

                robotPose.setPose(new Pose2D(DistanceUnit.INCH, newX, newY, AngleUnit.RADIANS, newH));
                isLLActiveThisLoop = true;
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

        double dtNano = (System.nanoTime() - lastLoopTime);
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
        double currentAlphaDeg = (rOmegaDeg - lastROmegaDeg) / dtSec;

        double accelMag = Math.hypot(currentAx, currentAy);
        lastRawVxField = turretVx;
        lastRawVyField = turretVy;
        lastROmegaDeg = rOmegaDeg;

        if (accelMag > MAX_PHYSICAL_ACCEL) {
            isImpactDetected = true;
            impactTimer.reset();
        } else if (impactTimer.milliseconds() > IMPACT_COOLDOWN_MS) {
            isImpactDetected = false;
        }

        double alpha = isImpactDetected ? ALPHA_IMPACT : ALPHA_NORMAL;
        double smoothVx = lastSmoothVxField * (1.0 - alpha) + turretVx * alpha;
        double smoothVy = lastSmoothVyField * (1.0 - alpha) + turretVy * alpha;

        lastSmoothAxField = lastSmoothAxField * (1.0 - alpha) + currentAx * alpha;
        lastSmoothAyField = lastSmoothAyField * (1.0 - alpha) + currentAy * alpha;

        lastSmoothVxField = smoothVx;
        lastSmoothVyField = smoothVy;
        lastLoopTime = System.nanoTime();

        AimCalculator.AimResult aim = AimCalculator.solveAim(
                turretX, turretY, smoothVx, smoothVy, lastSmoothAxField, lastSmoothAyField, targetX, targetY
        );

        double distanceToTarget = Math.hypot(targetX - turretX, targetY - turretY);
        double feedforwardPower = 0.0;

        dynamicToleranceDeg = Math.toDegrees(Math.atan2(TARGET_HITBOX_WIDTH_INCHES / 2.0, distanceToTarget));
        if(dynamicToleranceDeg > MAX_AIM_ANGLE_TOLERANCE) dynamicToleranceDeg = MAX_AIM_ANGLE_TOLERANCE;
        if(dynamicToleranceDeg < 0.5) dynamicToleranceDeg = 0.5;

        if (aim != null && turretMotor != null) {
            command.hasTarget = true;
            command.targetRpm = aim.rpm;
            command.targetPitch = aim.pitch;
            command.targetDistance = distanceToTarget;

            double targetAbsHeading = aim.algYaw - 90.0;
            double currentChassisHeading = robotPose.getHeading(AngleUnit.DEGREES);

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
            command.isUnwinding = Math.abs(command.pidErrorDeg) > 45.0;
            command.isAimLocked = Math.abs(command.pidErrorDeg) <= dynamicToleranceDeg;

            double requiredOmegaFromRotation = -rOmegaDeg;
            double dx = targetX - turretX;
            double dy = targetY - turretY;
            double distSq = dx * dx + dy * dy;
            double tangentialVelocity = (turretVx * dy - turretVy * dx) / Math.sqrt(distSq);
            double requiredOmegaFromTranslation = Math.toDegrees(tangentialVelocity / Math.sqrt(distSq));

            double totalRequiredRelativeOmega = requiredOmegaFromRotation + requiredOmegaFromTranslation;
            double requiredAlphaFromRotation = -currentAlphaDeg;
            feedforwardPower = (totalRequiredRelativeOmega * kV_TURRET) + (requiredAlphaFromRotation * kA_TURRET);
        }

        if (turretMotor != null) {
            if (command.hasTarget) {
                double error = command.pidErrorDeg;
                double absError = Math.abs(error);
                double dt = pidTimer.seconds();
                if (dt == 0) dt = 0.001;

                double pidPower = 0.0;

                if (absError <= ERROR_TOLERANCE) {
                    pidPower = 0;
                    integralSum = 0;
                    currentStageName = "【到达目标】";
                    wasInStage1 = false;
                }
                else if (absError > STAGE_THRESHOLD) {
                    currentStageName = "【第一段】(远距)";
                    wasInStage1 = true;

                    integralSum += error * dt;
                    double derivative = (error - lastError) / dt;

                    pidPower = (kP_far * error) + (kI_far * integralSum) + (kD_far * derivative);
                }
                else {
                    currentStageName = "【第二段】(近距)";

                    if (wasInStage1 || Math.signum(error) != Math.signum(lastError)) {
                        integralSum = 0;
                        wasInStage1 = false;
                    }

                    if (absError < I_ZONE_near) {
                        integralSum += error * dt;
                        double maxISum = MAX_INTEGRAL_POWER / (kI_near == 0 ? 1 : kI_near);
                        integralSum = Math.max(-maxISum, Math.min(maxISum, integralSum));
                    } else {
                        integralSum = 0;
                    }

                    double derivative = (error - lastError) / dt;

                    double frictionFF = Math.signum(error) * kS_near;
                    pidPower = (kP_near * error) + (kI_near * integralSum) + (kD_near * derivative) + frictionFF;
                }

                double finalPower = pidPower + feedforwardPower;
                finalPower = Math.max(-1.0, Math.min(1.0, finalPower));

                turretMotor.setPower(finalPower);
                currentTurretPower = finalPower;

                lastError = error;
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

        printTelemetry(aim, command, rX, rY, targetX, targetY, feedforwardPower);

        return command;
    }

    private void printTelemetry(AimCalculator.AimResult aim, TurretCommand command, double rX, double rY, double targetX, double targetY, double ffPower) {
        telemetry.addData("AutoAim Status", isImpactDetected ? "[! IMPACT !]" : "[ OK ]");
        telemetry.addData("Chassis Pos", "X:%.1f  Y:%.1f", rX, rY);
        telemetry.addData("Vision Filter", isLLActiveThisLoop ? "ACTIVE (Moving/Stationary Fusion)" : "DISABLED (Too far)");

        if(aim != null && command.hasTarget) {
            telemetry.addData("Target Locked", "WorldX:%.0f WorldY:%.0f | Dist: %.1f in", targetX, targetY, command.targetDistance);
            telemetry.addData("Aim Command", "RPM: %.0f | Pitch: %.2f", aim.rpm, aim.pitch);

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
            telemetry.addData("[System]", "Odometry Pose Overridden to X: " + (-pose.getX(DistanceUnit.INCH)));
        }
    }
}