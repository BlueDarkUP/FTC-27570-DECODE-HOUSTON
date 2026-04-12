package org.firstinspires.ftc.teamcode.AutoAim;

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

public class AutoAimSubsystem {

    private PinpointPoseProvider robotPose;
    private PinpointPoseProvider turretPose;
    private Limelight3A ll;
    private Telemetry telemetry;
    private DcMotorEx turretMotor;

    public final double FIELD_OFFSET_X = 72.0;
    public final double FIELD_OFFSET_Y = 72.0;
    public final double TURRET_OFFSET_FWD = -2.5;
    public final double TURRET_OFFSET_LEFT = 0.0;

    public final double TURRET_SOFT_LIMIT_CCW = 175.0;
    public final double TURRET_SOFT_LIMIT_CW = -210.0;

    public final double TURRET_START_OFFSET_DEG = 0;
    public final double TURRET_TICKS_PER_REV = 32798;
    public final double TICKS_PER_DEGREE = TURRET_TICKS_PER_REV / 360.0;

    public final double STATIONARY_SPEED_LIMIT = 5.0;
    public final double MAX_PHYSICAL_ACCEL = 10000;
    public final double IMPACT_COOLDOWN_MS = 100;
    public final double ALPHA_NORMAL = 0.80;
    public final double ALPHA_IMPACT = 0.05;

    public final double LL_FILTER_WEIGHT = 0.7;
    public final double LL_MAX_TRUST_DISTANCE = 120.0;

    // ==========================================================
    // === 第一段 PID 参数 (远距离：大开大合，追求速度，不管摩擦) ===
    // ==========================================================
    private final double STAGE_THRESHOLD = 14.0;
    private final double kP_far = 0.05;
    private final double kI_far = 0.00;
    private final double kD_far = 0.0003;

    // ==========================================================
    // === 第二段 PID 参数 (近距离：精雕细琢，克服摩擦，防止超调) ===
    // ==========================================================
    private final double kP_near = 0.03;
    private final double kI_near = 0.00005;
    private final double kD_near = 0.000001;

    private final double kS_near = 0.05;
    private final double I_ZONE_near = 3.0;
    private final double MAX_INTEGRAL_POWER = 0.08;

    // ==========================================================
    // === 终点死区 ===
    // ==========================================================
    private final double ERROR_TOLERANCE = 0.5;

    private long lastLoopTime = 0;
    private double lastRawVxField = 0;
    private double lastRawVyField = 0;
    private double lastSmoothVxField = 0;
    private double lastSmoothVyField = 0;
    private boolean isImpactDetected = false;
    private ElapsedTime impactTimer = new ElapsedTime();

    // 双段 PID 状态变量
    private double integralSum = 0;
    private double lastError = 0;
    private boolean wasInStage1 = false;
    private String currentStageName = "IDLE";

    private ElapsedTime pidTimer = new ElapsedTime();
    private double currentTurretPower = 0;

    private boolean isLLActiveThisLoop = false;

    public static class TurretCommand {
        public boolean hasTarget = false;
        public double pidErrorDeg = 0;
        public double targetRpm = 0;
        public double targetPitch = 0;
        public double targetDistance = 0;
        public boolean isUnwinding = false;
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
        double rOmega = robotPose.getHeadingVelocity(AngleUnit.RADIANS);
        double speed = Math.hypot(rVx, rVy);

        double chassisDistToTarget = Math.hypot(targetX - rX, targetY - rY);

        isLLActiveThisLoop = false;
        if (ll != null) {
            LLResult result = ll.getLatestResult();
            if (result != null && result.isValid() && speed < STATIONARY_SPEED_LIMIT && chassisDistToTarget < LL_MAX_TRUST_DISTANCE) {
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

                double newX = currentOdoX + (targetLLPose.getX(DistanceUnit.INCH) - currentOdoX) * LL_FILTER_WEIGHT;
                double newY = currentOdoY + (targetLLPose.getY(DistanceUnit.INCH) - currentOdoY) * LL_FILTER_WEIGHT;

                double angleDiff = AngleUnit.normalizeRadians(targetLLPose.getHeading(AngleUnit.RADIANS) - currentOdoH);
                double newH = AngleUnit.normalizeRadians(currentOdoH + (angleDiff * LL_FILTER_WEIGHT));

                robotPose.setPose(new Pose2D(DistanceUnit.INCH, newX, newY, AngleUnit.RADIANS, newH));
                isLLActiveThisLoop = true;
            }
        }

        double cosH = Math.cos(rH_Rad);
        double sinH = Math.sin(rH_Rad);
        double turretX = rX + (TURRET_OFFSET_FWD * (-sinH)) + (TURRET_OFFSET_LEFT * (-cosH));
        double turretY = rY + (TURRET_OFFSET_FWD * ( cosH)) + (TURRET_OFFSET_LEFT * (-sinH));
        double tanVx = (TURRET_OFFSET_FWD * -cosH * rOmega) + (TURRET_OFFSET_LEFT * sinH * rOmega);
        double tanVy = (TURRET_OFFSET_FWD * -sinH * rOmega) + (TURRET_OFFSET_LEFT * -cosH * rOmega);
        double turretVx = rVx + tanVx;
        double turretVy = rVy + tanVy;

        if (lastLoopTime == 0) {
            lastLoopTime = System.nanoTime();
            lastRawVxField = turretVx;
            lastRawVyField = turretVy;
            lastSmoothVxField = turretVx;
            lastSmoothVyField = turretVy;
        }

        double dtNano = (System.nanoTime() - lastLoopTime) / 1.0E9;
        if(dtNano < 0.001) dtNano = 0.001;
        double accel = Math.hypot((turretVx - lastRawVxField)/dtNano, (turretVy - lastRawVyField)/dtNano);
        lastRawVxField = turretVx;
        lastRawVyField = turretVy;

        if (accel > MAX_PHYSICAL_ACCEL) {
            isImpactDetected = true;
            impactTimer.reset();
        } else if (impactTimer.milliseconds() > IMPACT_COOLDOWN_MS) {
            isImpactDetected = false;
        }

        double alpha = isImpactDetected ? ALPHA_IMPACT : ALPHA_NORMAL;
        double smoothVx = lastSmoothVxField * (1.0 - alpha) + turretVx * alpha;
        double smoothVy = lastSmoothVyField * (1.0 - alpha) + turretVy * alpha;
        lastSmoothVxField = smoothVx;
        lastSmoothVyField = smoothVy;
        lastLoopTime = System.nanoTime();

        AimCalculator.AimResult aim = AimCalculator.solveAim(
                turretX, turretY, smoothVx, smoothVy, targetX, targetY
        );

        double distanceToTarget = Math.hypot(targetX - turretX, targetY - turretY);

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
        }

        if (turretMotor != null) {
            if (command.hasTarget) {
                double error = command.pidErrorDeg;
                double absError = Math.abs(error);
                double dt = pidTimer.seconds();
                if (dt == 0) dt = 0.001;

                double power = 0.0;

                // ================= 双段 PID 核心逻辑接入 =================

                // 【状态机 1：到达死区】
                if (absError <= ERROR_TOLERANCE) {
                    power = 0;
                    integralSum = 0;
                    currentStageName = "【到达目标】";
                    wasInStage1 = false;
                }
                // 【状态机 2：第一段 (远距离)】
                else if (absError > STAGE_THRESHOLD) {
                    currentStageName = "【第一段】(远距)";
                    wasInStage1 = true;

                    integralSum += error * dt;
                    double derivative = (error - lastError) / dt;

                    power = (kP_far * error) + (kI_far * integralSum) + (kD_far * derivative);
                }
                // 【状态机 3：第二段 (近距离精调)】
                else {
                    currentStageName = "【第二段】(近距)";

                    // 阶段切换或穿过目标点时重置积分
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
                    double pidPower = (kP_near * error) + (kI_near * integralSum) + (kD_near * derivative);

                    // 静摩擦力前馈
                    double feedforward = Math.signum(error) * kS_near;

                    power = pidPower + feedforward;
                }

                power = Math.max(-1.0, Math.min(1.0, power));
                turretMotor.setPower(power);
                currentTurretPower = power;

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

        printTelemetry(aim, command, rX, rY, targetX, targetY);

        return command;
    }

    private void printTelemetry(AimCalculator.AimResult aim, TurretCommand command, double rX, double rY, double targetX, double targetY) {
        telemetry.addData("AutoAim Status", isImpactDetected ? "[! IMPACT !]" : "[ OK ]");
        telemetry.addData("Chassis Pos", "X:%.1f  Y:%.1f", rX, rY);
        telemetry.addData("Vision Filter", isLLActiveThisLoop ? "ACTIVE (Smoothing Drift)" : "DISABLED (Too far/Moving)");

        if(aim != null && command.hasTarget) {
            telemetry.addData("Target Locked", "WorldX:%.0f WorldY:%.0f | Dist: %.1f in", targetX, targetY, command.targetDistance);
            telemetry.addData("Aim Command", "RPM: %.0f | Pitch: %.2f", aim.rpm, aim.pitch);

            double encDeg = turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;

            // 新增了阶段名称的输出，方便在主 TeleOp 中随时监控云台状态
            telemetry.addData("Turret Control", "%s Pow: %.2f | Err: %.1f° | Tgt: %.1f°",
                    currentStageName, currentTurretPower, command.pidErrorDeg, (encDeg + command.pidErrorDeg));

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