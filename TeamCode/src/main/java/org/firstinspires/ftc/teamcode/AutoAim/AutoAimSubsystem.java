package org.firstinspires.ftc.teamcode.AutoAim;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class AutoAimSubsystem {

    private DcMotorEx Turret;
    private Servo LP;
    private Servo RP;

    // PID 参数：负责微调和最终锁定的精准度
    public static double TURRET_kP = 55.0;
    public static double TURRET_kI = 0.0;
    public static double TURRET_kD = 0.3;
    public static double TURRET_kF = 0.0001;

    // 基础前馈：kV 对应速度，kS 对应静摩擦
    public static double TURRET_kV = 0.001;
    public static double TURRET_kS = 0.1;

    public static double TURRET_kA = 0.0001;

    public static double TURRET_LATENCY = 0.01;

    // 死区与最大功率
    public static double TURRET_DEADZONE_DEG = 0.8;
    public static double TURRET_MAX_POWER = 1.0;

    // 滤波器系数
    public static double TURRET_FILTER_ALPHA = 0.7;
    public static double TURRET_VEL_FILTER_ALPHA = 0.9;

    // 预测刹车控制常数 (由 TurretPredictiveBrakingTuner 测得)
    public static double TURRET_kLinearBraking = 0.01765;
    public static double TURRET_kQuadraticFriction = 0.000086;

    private PIDFController turretPIDF;
    private final double TICKS_PER_REV = 32798.0;

    // 俯仰角伺服参数
    private final double LP_UP = 1.0;
    private final double LP_DOWN = 0.4;
    private final double RP_UP = 0.0;
    private final double RP_DOWN = 0.5;

    // 状态存储变量
    private double filteredTurretRelAngle = 0.0;
    private boolean isFilterInitialized = false;

    private double lastTurretRelAngle = 0.0;
    private double filteredTurretVel = 0.0;
    private double lastTargetVel = 0.0; // 用于计算加速度前馈
    private long lastTime = 0;

    public static class TurretCommand {
        public boolean hasTarget = false;
        public double targetRpm = 0.0;
        public double targetPitch = 0.0;
        public boolean isAimLocked = false;
        public boolean isUnwinding = false;
        public double currentTolerance = 1.0; // 新增：用于 TeleOp 显示当前的容错阈值
    }

    public AutoAimSubsystem(HardwareMap hardwareMap) {
        Turret = hardwareMap.get(DcMotorEx.class, "Turret");
        Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LP = hardwareMap.get(Servo.class, "LP");
        RP = hardwareMap.get(Servo.class, "RP");

        turretPIDF = new PIDFController(TURRET_kP, TURRET_kI, TURRET_kD, TURRET_kF);

        setPitchServos(0.7);
    }

    private void setPitchServos(double targetPitch) {
        double clampedLP = Math.max(LP_DOWN, Math.min(LP_UP, targetPitch));
        double proportion = (clampedLP - LP_DOWN) / (LP_UP - LP_DOWN);
        double calculatedRP = RP_DOWN + proportion * (RP_UP - RP_DOWN);
        calculatedRP = Math.max(0.0, Math.min(1.0, calculatedRP));

        LP.setPosition(clampedLP);
        RP.setPosition(calculatedRP);
    }

    public TurretCommand update(
            double robotX, double robotY, double globalVx, double globalVy,
            double currentHeadingDeg, double robotAngularVelocityDeg,
            double targetX, double targetY,
            boolean isManualMode, double manualDist) {

        turretPIDF.setPIDF(TURRET_kP, TURRET_kI, TURRET_kD, TURRET_kF);

        TurretCommand command = new TurretCommand();
        AimCalculator.AimResult aimResult;

        if (isManualMode) {
            double manualRpm = AimCalculator.interpolate(manualDist, 1);
            double manualPitch = AimCalculator.interpolate(manualDist, 2);
            aimResult = new AimCalculator.AimResult(manualDist, currentHeadingDeg, manualRpm, manualPitch, 0.0);
        } else {
            aimResult = AimCalculator.solveAim(
                    robotX, robotY, globalVx, globalVy, 0, 0, targetX, targetY
            );
        }

        if (aimResult != null) {
            command.hasTarget = true;
            command.targetRpm = aimResult.rpm;
            command.targetPitch = aimResult.pitch;

            // --- 线性变化容错计算 (20in -> 5deg, 150in -> 1deg) ---
            // 公式：y = y1 + (y2 - y1) / (x2 - x1) * (x - x1)
            double calculatedTolerance = 5.0 + (1.0 - 5.0) / (150.0 - 20.0) * (aimResult.dist - 20.0);
            // 限制范围在 1.0 到 5.0 之间
            command.currentTolerance = Math.max(1.0, Math.min(5.0, calculatedTolerance));

            // --- 1. 时间、角度读取与低通滤波 ---
            long currentTime = System.nanoTime();
            double dt = (lastTime == 0) ? 0 : (currentTime - lastTime) / 1e9;
            lastTime = currentTime;

            double currentTurretTicks = Turret.getCurrentPosition();
            double rawTurretRelAngle = (currentTurretTicks / TICKS_PER_REV) * 360.0;

            if (!isFilterInitialized) {
                filteredTurretRelAngle = rawTurretRelAngle;
                lastTurretRelAngle = rawTurretRelAngle;
                filteredTurretVel = 0.0;
                lastTargetVel = 0.0;
                isFilterInitialized = true;
            } else {
                filteredTurretRelAngle = (TURRET_FILTER_ALPHA * rawTurretRelAngle) + ((1.0 - TURRET_FILTER_ALPHA) * filteredTurretRelAngle);
                if (dt > 0.0001) {
                    double rawVel = (rawTurretRelAngle - lastTurretRelAngle) / dt;
                    filteredTurretVel = (TURRET_VEL_FILTER_ALPHA * rawVel) + ((1.0 - TURRET_VEL_FILTER_ALPHA) * filteredTurretVel);
                }
            }
            lastTurretRelAngle = rawTurretRelAngle;

            // --- 2. 运动学前馈计算 (Translational Kinematic FF) ---
            double dx = targetX - robotX;
            double dy = targetY - robotY;
            double distSq = dx * dx + dy * dy;
            double translationalOmegaDeg = 0.0;

            if (distSq > 0.001) {
                // 计算目标相对于机器人的角速度 (World Frame)
                double omegaRad = (-dx * globalVy + dy * globalVx) / distSq;
                translationalOmegaDeg = Math.toDegrees(omegaRad);
            }

            // --- 3. 延迟补偿逻辑 (Latency Compensation) ---
            double compensatedTargetAbsAngle = aimResult.algYaw + (translationalOmegaDeg * TURRET_LATENCY);

            // --- 4. 误差计算与 Unwind 逻辑 ---
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

            // 使用计算出的动态容错阈值进行判定
            command.isAimLocked = Math.abs(error) <= command.currentTolerance;

            // --- 5. 预测刹车 (仅在 Unwinding 时启用) ---
            double brakingDist = 0.0;
            double predictedRelAngle = filteredTurretRelAngle;
            if (command.isUnwinding) {
                brakingDist = (TURRET_kLinearBraking * Math.abs(filteredTurretVel))
                        + (TURRET_kQuadraticFriction * filteredTurretVel * filteredTurretVel);
                predictedRelAngle = filteredTurretRelAngle + (Math.signum(filteredTurretVel) * brakingDist);
            }

            // --- 6. 动力融合 (PID + kV + kA + kS) ---
            double pidOutputVel = turretPIDF.calculate(predictedRelAngle, targetTurretRelAngle);
            double feedforwardVel = -robotAngularVelocityDeg + translationalOmegaDeg;
            double finalTargetVel = pidOutputVel + feedforwardVel;

            double targetAccel = 0.0;
            if (dt > 0.0001) {
                targetAccel = (finalTargetVel - lastTargetVel) / dt;
            }
            lastTargetVel = finalTargetVel;

            double turretPower;
            if (Math.abs(error) < TURRET_DEADZONE_DEG && Math.abs(robotAngularVelocityDeg) < 3.0 && Math.abs(translationalOmegaDeg) < 5.0) {
                turretPower = 0.0;
                turretPIDF.reset();
                lastTargetVel = 0.0;
            } else {
                turretPower = (targetAccel * TURRET_kA)
                        + (finalTargetVel * TURRET_kV)
                        + (Math.signum(finalTargetVel) * TURRET_kS);
            }

            turretPower = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, turretPower));

            Turret.setPower(turretPower);
            setPitchServos(command.targetPitch);

            // --- 7. Dashboard 遥测 ---
            TelemetryPacket packet = new TelemetryPacket();
            drawAnalysis(packet.fieldOverlay(), robotX, robotY, currentHeadingDeg, filteredTurretRelAngle, aimResult.algYaw, targetX, targetY);

            packet.put("Turret/Error", error);
            packet.put("Turret/CurrentTolerance", command.currentTolerance); // 方便在 Dashboard 观察当前容错
            packet.put("Turret/TargetVel", finalTargetVel);
            packet.put("Turret/IsUnwinding", command.isUnwinding);

            FtcDashboard.getInstance().sendTelemetryPacket(packet);

        } else {
            Turret.setPower(0);
            isFilterInitialized = false;
        }

        return command;
    }

    private void drawAnalysis(Canvas canvas, double x, double y, double heading, double turretRel, double targetAbs, double tx, double ty) {
        canvas.setStrokeWidth(1);
        canvas.setStroke("blue");
        canvas.strokeCircle(x, y, 9);
        canvas.setStroke("green");
        canvas.strokeLine(x, y, tx, ty);
        double actualAbsRad = Math.toRadians(heading + turretRel);
        double lineLen = 30.0;
        canvas.setStroke("red");
        canvas.strokeLine(x, y, x + Math.cos(actualAbsRad) * lineLen, y + Math.sin(actualAbsRad) * lineLen);
        canvas.setStroke("yellow");
        canvas.strokeCircle(tx, ty, 2);
    }

    public void stop() {
        Turret.setPower(0);
        isFilterInitialized = false;
    }
}