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

    public final double TURRET_SOFT_LIMIT = 250.0;
    public final double TURRET_START_OFFSET_DEG = 180.0; // 新增：启动时云台相对于车头正前方的角度
    public final double TURRET_TICKS_PER_REV = 32798;
    public final double TICKS_PER_DEGREE = TURRET_TICKS_PER_REV / 360.0;

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
        public double targetDistance = 0; // 新增：距离目标的绝对直线距离(英寸)
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

        // 5. 弹道解算
        AimCalculator.AimResult aim = AimCalculator.solveAim(
                turretX, turretY, smoothVx, smoothVy, targetX, targetY
        );

        // 计算并记录目标距离
        double distanceToTarget = Math.hypot(targetX - turretX, targetY - turretY);

        // ================= 【核心修改】6. 软限位与长路径防缠绕解算 =================
        if (aim != null && turretMotor != null) {
            command.hasTarget = true;
            command.targetRpm = aim.rpm;
            command.targetPitch = aim.pitch;
            command.targetDistance = distanceToTarget; // 赋值距离到指令包

            // 1. 获取目标绝对朝向和底盘绝对朝向
            double targetAbsHeading = aim.algYaw - 90.0;
            double currentChassisHeading = robotPose.getHeading(AngleUnit.DEGREES);

            // 2. 获取编码器的原生连续角度 (范围大约在 -190 到 190 之间)
            double encoderDeg = turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;

            // 3. 计算当前云台相对于底盘的真实几何朝向 (核心修正：编码器角度 + 初始的180度偏移)
            double currentTurretRelDegToChassis = encoderDeg + TURRET_START_OFFSET_DEG;

            // 4. 计算想要到达的理想相对底盘角度
            double desiredRelDegToChassis = targetAbsHeading - currentChassisHeading;

            // 5. 计算最短角度误差 (目标几何角度 - 当前几何角度)
            double shortestPathError = AngleUnit.normalizeDegrees(desiredRelDegToChassis - currentTurretRelDegToChassis);

            // 6. 将误差映射回编码器的物理坐标系，得到未来的目标编码器角度
            double targetEncoderDeg = encoderDeg + shortestPathError;

            // 7. 防缠绕/突破软限位逻辑
            // 此时 targetEncoderDeg 完全对应软限位坐标系，可以直接与 190 比较
            if (targetEncoderDeg > TURRET_SOFT_LIMIT) {
                targetEncoderDeg -= 360.0; // 走远路绕回去
            } else if (targetEncoderDeg < -TURRET_SOFT_LIMIT) {
                targetEncoderDeg += 360.0; // 走远路绕回去
            }

            // 8. 死区保护：如果绕回去了依然在死区（比如目标正好在车头正前方狭窄的死区内），强行停在限位处
            if (targetEncoderDeg > TURRET_SOFT_LIMIT) targetEncoderDeg = TURRET_SOFT_LIMIT;
            if (targetEncoderDeg < -TURRET_SOFT_LIMIT) targetEncoderDeg = -TURRET_SOFT_LIMIT;

            // 9. 最终送给PID的误差值 = 目标编码器角度 - 当前编码器角度
            command.pidErrorDeg = targetEncoderDeg - encoderDeg;
        }

        // ================= 内置 PID 闭环控制 =================
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
                turretMotor.setPower(0);
                currentTurretPower = 0;
                integralSum = 0;
                lastError = 0;
                pidTimer.reset();
            }
        }
        // =============================================================

        // 7. 自动打印调试信息 (修改签名，传入包含距离信息的 command)
        printTelemetry(aim, command, rX, rY, targetX, targetY);

        return command;
    }

    private void printTelemetry(AimCalculator.AimResult aim, TurretCommand command, double rX, double rY, double targetX, double targetY) {
        telemetry.addData("AutoAim Status", isImpactDetected ? "[! IMPACT !]" : "[ OK ]");
        telemetry.addData("Chassis Pos", "X:%.1f  Y:%.1f", rX, rY);
        if(aim != null && command.hasTarget) {
            telemetry.addData("Target Locked", "WorldX:%.0f WorldY:%.0f | Dist: %.1f in", targetX, targetY, command.targetDistance);
            telemetry.addData("Aim Command", "RPM: %.0f | Pitch: %.2f", aim.rpm, aim.pitch);
            telemetry.addData("Turret Control", "Pow: %.2f | Err: %.1f°", currentTurretPower, command.pidErrorDeg);

            // 增加打印编码器物理角度和换算后的底盘相对几何角度，以便于验证
            double encDeg = turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;
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