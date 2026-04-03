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
import org.firstinspires.ftc.teamcode.vision.EchoLapse.PinpointPoseProvider;

public class AutoAimSubsystem {

    // ================= 硬件声明 =================
    private PinpointPoseProvider robotPose;
    private PinpointPoseProvider turretPose;
    private Limelight3A ll;
    private Telemetry telemetry;
    private DcMotorEx turretMotor;

    // ================= 常量配置 =================
    public final double FIELD_OFFSET_X = 72.0;
    public final double FIELD_OFFSET_Y = 72.0;
    public final double TURRET_OFFSET_FWD = -2.0;
    public final double TURRET_OFFSET_LEFT = 0.0;
    public final double TURRET_SOFT_LIMIT = 190.0;
    public final double STATIONARY_SPEED_LIMIT = 5.0;
    public final double MAX_PHYSICAL_ACCEL = 1000;
    public final double IMPACT_COOLDOWN_MS = 300.0;
    public final double ALPHA_NORMAL = 0.80;
    public final double ALPHA_IMPACT = 0.05;

    private final double kP = 0.035;
    private final double kI = 0.000;
    private final double kD = 0.00145;

    // ================= 状态变量 =================
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

    // ================= 返回值数据结构 =================
    public static class TurretCommand {
        public boolean hasTarget = false;
        public double pidErrorDeg = 0;
        public double targetRpm = 0;
        public double targetPitch = 0;
    }

    /**
     * 构造函数：初始化所有硬件
     */
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
            turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
            turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        } catch (Exception e) {
            telemetry.addLine("[FATAL] Turret Motor missing.");
        }

        pidTimer.reset();
    }

    /**
     * 主循环调用方法：计算瞄准参数并自动控制云台电机
     * @param targetX 目标世界坐标X
     * @param targetY 目标世界坐标Y
     * @return TurretCommand 包含所需的控制指令（给摩擦轮和Pitch舵机留作后用）
     */
    public TurretCommand update(double targetX, double targetY) {
        TurretCommand command = new TurretCommand();
        if (robotPose == null) return command;

        if (lastLoopTime == 0) lastLoopTime = System.nanoTime();

        // 1. 读取位置传感器
        robotPose.update();
        if (turretPose != null) turretPose.update();

        double rX = -robotPose.getX(DistanceUnit.INCH);
        double rY = robotPose.getY(DistanceUnit.INCH);
        double rH_Rad = robotPose.getHeading(AngleUnit.RADIANS);
        double rVx = robotPose.getXVelocity(DistanceUnit.INCH);
        double rVy = robotPose.getYVelocity(DistanceUnit.INCH);
        double rOmega = robotPose.getHeadingVelocity(AngleUnit.RADIANS);
        double speed = Math.hypot(rVx, rVy);

        // 2. 视觉辅助定位融合
        if (ll != null) {
            LLResult result = ll.getLatestResult();
            if (result != null && result.isValid() && speed < STATIONARY_SPEED_LIMIT) {
                Pose3D botpose = result.getBotpose();
                double llRawX_Meters = botpose.getPosition().x;
                double llRawY_Meters = botpose.getPosition().y;
                double llRawYaw_Rad = botpose.getOrientation().getYaw(AngleUnit.RADIANS);

                double targetWorldX_Inches = (llRawY_Meters * 39.3701) + FIELD_OFFSET_X;
                double targetWorldY_Inches = (-llRawX_Meters * 39.3701) + FIELD_OFFSET_Y;
                double mappedHeading_Rad = AngleUnit.normalizeRadians(llRawYaw_Rad + Math.PI);

                double latency = (result.getCaptureLatency() + result.getTargetingLatency()) / 1000.0;
                Pose2D correctedPose = new Pose2D(DistanceUnit.INCH,
                        targetWorldX_Inches + (rVx * latency),
                        targetWorldY_Inches + (rVy * latency),
                        AngleUnit.RADIANS, mappedHeading_Rad);

                robotPose.setPose(correctedPose);
            }
        }

        // 3. 云台运动学换算
        double cosH = Math.cos(rH_Rad);
        double sinH = Math.sin(rH_Rad);
        double turretX = rX + (TURRET_OFFSET_FWD * (-sinH)) + (TURRET_OFFSET_LEFT * (-cosH));
        double turretY = rY + (TURRET_OFFSET_FWD * ( cosH)) + (TURRET_OFFSET_LEFT * (-sinH));
        double tanVx = (TURRET_OFFSET_FWD * -cosH * rOmega) + (TURRET_OFFSET_LEFT * sinH * rOmega);
        double tanVy = (TURRET_OFFSET_FWD * -sinH * rOmega) + (TURRET_OFFSET_LEFT * -cosH * rOmega);
        double turretVx = rVx + tanVx;
        double turretVy = rVy + tanVy;

        // 4. 撞击检测与加速度滤波
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

        // 5. 弹道解算
        AimCalculator.AimResult aim = AimCalculator.solveAim(
                turretX, turretY, smoothVx, smoothVy, targetX, targetY
        );

        // 6. 软限位与最短路径安全计算
        if (aim != null) {
            command.hasTarget = true;
            command.targetRpm = aim.rpm;
            command.targetPitch = aim.pitch;

            double targetAbsHeading = aim.algYaw - 90.0;
            double currentChassisHeading = robotPose.getHeading(AngleUnit.DEGREES);
            double currentGunAbsHeading = turretPose != null ? turretPose.getHeading(AngleUnit.DEGREES) : currentChassisHeading;
            double turretRelDeg = AngleUnit.normalizeDegrees(currentGunAbsHeading - currentChassisHeading);

            double rawTurnDiff = AngleUnit.normalizeDegrees(targetAbsHeading - currentGunAbsHeading);
            double futurePosition = turretRelDeg + rawTurnDiff;

            // 线缆缠绕与死区保护
            if (futurePosition > TURRET_SOFT_LIMIT) futurePosition -= 360.0;
            else if (futurePosition < -TURRET_SOFT_LIMIT) futurePosition += 360.0;

            if (futurePosition > TURRET_SOFT_LIMIT) futurePosition = TURRET_SOFT_LIMIT;
            if (futurePosition < -TURRET_SOFT_LIMIT) futurePosition = -TURRET_SOFT_LIMIT;

            command.pidErrorDeg = futurePosition - turretRelDeg;
        }

        // ================= 【核心新增】内置 PID 闭环控制 =================
        if (turretMotor != null) {
            if (command.hasTarget) {
                double error = command.pidErrorDeg;
                double dt = pidTimer.seconds();
                if (dt == 0) dt = 0.001;

                integralSum += error * dt;
                double derivative = (error - lastError) / dt;

                double power = (kP * error) + (kI * integralSum) + (kD * derivative);
                power = Math.max(-1.0, Math.min(1.0, power)); // 限制在 [-1, 1] 之间

                turretMotor.setPower(power);
                currentTurretPower = power;

                lastError = error;
                pidTimer.reset();
            } else {
                // 如果丢失目标，断电清空缓存防止 Windup
                turretMotor.setPower(0);
                currentTurretPower = 0;
                integralSum = 0;
                lastError = 0;
                pidTimer.reset();
            }
        }
        // =============================================================

        // 7. 自动打印调试信息
        printTelemetry(aim, command.pidErrorDeg, rX, rY, targetX, targetY);

        return command;
    }

    private void printTelemetry(AimCalculator.AimResult aim, double error, double rX, double rY, double targetX, double targetY) {
        telemetry.addData("AutoAim Status", isImpactDetected ? "[! IMPACT !]" : "[ OK ]");
        telemetry.addData("Chassis Pos", "X:%.1f  Y:%.1f", rX, rY);
        if(aim != null) {
            telemetry.addData("Target Locked", "WorldX:%.0f WorldY:%.0f", targetX, targetY);
            telemetry.addData("Aim Command", "RPM: %.0f | Pitch: %.2f", aim.rpm, aim.pitch);
            telemetry.addData("Turret Control", "Pow: %.2f | Err: %.1f°", currentTurretPower, error);
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