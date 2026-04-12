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
    public final double TURRET_OFFSET_FWD = -2.0;
    public final double TURRET_OFFSET_LEFT = 0.0;

    public final double TURRET_SOFT_LIMIT_CCW = 175.0;
    public final double TURRET_SOFT_LIMIT_CW = -210.0;

    public final double TURRET_START_OFFSET_DEG = 0;
    public final double TURRET_TICKS_PER_REV = 32798;
    public final double TICKS_PER_DEGREE = TURRET_TICKS_PER_REV / 360.0;

    public final double STATIONARY_SPEED_LIMIT = 5.0;
    public final double MAX_PHYSICAL_ACCEL = 1000;
    public final double IMPACT_COOLDOWN_MS = 300.0;
    public final double ALPHA_NORMAL = 0.80;
    public final double ALPHA_IMPACT = 0.05;

    public final double LL_FILTER_WEIGHT = 0.1;
    public final double LL_MAX_TRUST_DISTANCE = 90.0;

    private final double kP = 0.035;
    private final double kI = 0.001;
    private final double kD = 0.0015;

    private final double kS = 0.06;

    private final double I_ZONE = 8.0;

    private final double MAX_INTEGRAL_POWER = 0.2;
    private final double ERROR_TOLERANCE = 0.8;

    private long lastLoopTime = 0;
    private double lastRawVxField = 0;
    private double lastRawVyField = 0;
    private double lastSmoothVxField = 0;
    private double lastSmoothVyField = 0;
    private boolean isImpactDetected = false;
    private ElapsedTime impactTimer = new ElapsedTime();

    private double integralSum = 0;
    private double lastError = 0;
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
        double rVx = robotPose.getXVelocity(DistanceUnit.INCH);
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
                double dt = pidTimer.seconds();
                if (dt == 0) dt = 0.001;

                // 【核心优化 1】死区控制 (Deadband)
                if (Math.abs(error) <= ERROR_TOLERANCE) {
                    // 进入死区，视为到达目标，切断动力并清空积分
                    turretMotor.setPower(0);
                    currentTurretPower = 0;
                    integralSum = 0;
                } else {
                    // 【核心优化 2】过零清空积分 (Zero-Crossing Clearing)
                    // 如果这次的误差和上次的误差符号相反，说明我们刚刚越过了目标点
                    if (Math.signum(error) != Math.signum(lastError)) {
                        integralSum = 0;
                    }

                    // 【核心优化 3】积分区间与限幅 (I-Zone & Anti-Windup)
                    if (Math.abs(error) < I_ZONE) {
                        integralSum += error * dt;
                        // 限制积分累加的最大效果，防止积分饱和
                        double maxISum = MAX_INTEGRAL_POWER / (kI == 0 ? 1 : kI);
                        integralSum = Math.max(-maxISum, Math.min(maxISum, integralSum));
                    } else {
                        // 如果误差很大，根本不进行积分累加
                        integralSum = 0;
                    }

                    double derivative = (error - lastError) / dt;

                    // 计算基础 PID 动力
                    double pidPower = (kP * error) + (kI * integralSum) + (kD * derivative);

                    // 【核心优化 4】静摩擦力前馈 (Static Friction Feedforward)
                    // Math.signum(error) 会返回 1.0 (正误差) 或 -1.0 (负误差)
                    double feedforward = Math.signum(error) * kS;

                    // 最终动力 = PID动力 + 克服静摩擦力的最小动力
                    double power = pidPower + feedforward;

                    power = Math.max(-1.0, Math.min(1.0, power));

                    turretMotor.setPower(power);
                    currentTurretPower = power;
                }

                lastError = error;
                pidTimer.reset();
            } else {
                turretMotor.setPower(0);
                currentTurretPower = 0;
                integralSum = 0;
                lastError = 0;
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

            telemetry.addData("Turret Control", "Pow: %.2f | Err: %.1f° | Tgt: %.1f°",
                    currentTurretPower, command.pidErrorDeg, (encDeg + command.pidErrorDeg));

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