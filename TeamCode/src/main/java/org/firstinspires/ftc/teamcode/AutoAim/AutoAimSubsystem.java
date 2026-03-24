package org.firstinspires.ftc.teamcode.AutoAim;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
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
    private DcMotorEx turretEncoder;
    private Telemetry telemetry;

    // ================= 常量配置 =================
    public final double FIELD_OFFSET_X = 72.0;
    public final double FIELD_OFFSET_Y = 72.0;
    public final double TURRET_OFFSET_FWD = -2.0;          // 以车旋转中心为原点建立标准笛卡尔平面坐标系，云台在y方向的位置偏移
    public final double TURRET_OFFSET_LEFT = 0.0;          // 以车旋转中心为原点建立标准笛卡尔平面坐标系，云台在x方向的位置偏移
    public final double TURRET_TICKS_PER_REV = 8192.0;     // 云台转一·整圈（360°）时，贯穿轴编码器增加的数值
    public final double TURRET_SOFT_LIMIT = 190.0;         // 硬限位角度-10
    public final double STATIONARY_SPEED_LIMIT = 5.0;      // 车速小于多少信赖ll
    public final double MAX_PHYSICAL_ACCEL = 1000;
    public final double IMPACT_COOLDOWN_MS = 300.0;
    public final double ALPHA_NORMAL = 0.80;               // 低通滤波器
    public final double ALPHA_IMPACT = 0.05;

    // ================= 状态变量 =================
    private long lastLoopTime = 0;
    private double lastRawVxField = 0;
    private double lastRawVyField = 0;
    private double lastSmoothVxField = 0;
    private double lastSmoothVyField = 0;
    private boolean isImpactDetected = false;
    private ElapsedTime impactTimer = new ElapsedTime();

    // ================= 返回值数据结构 =================
    public static class TurretCommand {
        public boolean hasTarget = false;
        public double pidErrorDeg = 0;   // 发送给云台PID控制器的角度误差
        public double targetRpm = 0;     // 发送给摩擦轮的目标RPM
        public double targetPitch = 0;   // 发送给Pitch舵机的目标位置
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
            turretPose = new PinpointPoseProvider(hardwareMap, "turretPinpoint");
            turretPose.initialize();
            turretEncoder = hardwareMap.get(DcMotorEx.class, "turretEncoder");
        } catch (Exception e) {
            telemetry.addLine("[WARN] Turret hardware missing. Virtual Turret Mode Active.");
        }
    }

    /**
     * 主循环调用方法：计算瞄准参数
     * @param targetX 目标世界坐标X
     * @param targetY 目标世界坐标Y
     * @return TurretCommand 包含所需的控制指令
     */
    public TurretCommand update(double targetX, double targetY) {
        TurretCommand command = new TurretCommand();
        if (robotPose == null) return command; // 里程计未就绪，直接返回

        if (lastLoopTime == 0) lastLoopTime = System.nanoTime();

        // 1. 读取位置传感器
        robotPose.update();
        if (turretPose != null) turretPose.update();

        double rX = robotPose.getX(DistanceUnit.INCH);
        double rY = robotPose.getY(DistanceUnit.INCH);
        double rH_Rad = robotPose.getHeading(AngleUnit.RADIANS);
        double rVx = robotPose.getXVelocity(DistanceUnit.INCH);
        double rVy = robotPose.getYVelocity(DistanceUnit.INCH);
        double rOmega = robotPose.getHeadingVelocity(AngleUnit.RADIANS);
        double speed = Math.hypot(rVx, rVy);

        // 2. 视觉辅助定位融合 (Limelight Relocalization)
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
            // 初始化历史速度，防止第一帧求导爆炸
            lastRawVxField = turretVx;
            lastRawVyField = turretVy;
            lastSmoothVxField = turretVx;
            lastSmoothVyField = turretVy;
        }

        double dt = (System.nanoTime() - lastLoopTime) / 1.0E9;
        if(dt < 0.001) dt = 0.001;
        double accel = Math.hypot((turretVx - lastRawVxField)/dt, (turretVy - lastRawVyField)/dt);
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
            double turretRelDeg = (turretEncoder != null) ? (turretEncoder.getCurrentPosition() / TURRET_TICKS_PER_REV * 360.0) : 0.0;

            double currentGunAbsHeading;
            if (isImpactDetected && turretPose != null) {
                currentGunAbsHeading = turretPose.getHeading(AngleUnit.DEGREES);
            } else {
                currentGunAbsHeading = currentChassisHeading + turretRelDeg;
            }

            double rawTurnDiff = AngleUnit.normalizeDegrees(targetAbsHeading - currentGunAbsHeading);
            double futurePosition = turretRelDeg + rawTurnDiff;

            // 线缆缠绕保护
            if (futurePosition > TURRET_SOFT_LIMIT) futurePosition -= 360.0;
            else if (futurePosition < -TURRET_SOFT_LIMIT) futurePosition += 360.0;

            // 死区截断保护
            if (futurePosition > TURRET_SOFT_LIMIT) futurePosition = TURRET_SOFT_LIMIT;
            if (futurePosition < -TURRET_SOFT_LIMIT) futurePosition = -TURRET_SOFT_LIMIT;

            command.pidErrorDeg = futurePosition - turretRelDeg;
        }

        // 7. （可选）自动打印调试信息，保持原有的 Telemetry 体验
        printTelemetry(aim, command.pidErrorDeg, rX, rY, accel);

        return command;
    }

    private void printTelemetry(AimCalculator.AimResult aim, double error, double rX, double rY, double accel) {
        telemetry.addData("AutoAim Status", isImpactDetected ? "[! IMPACT !]" : "[ OK ]");
        telemetry.addData("Chassis Pos", "X:%.1f  Y:%.1f", rX, rY);
        if(aim != null) {
            telemetry.addData("Aim Command", "RPM: %.0f | Pitch: %.2f", aim.rpm, aim.pitch);
            telemetry.addData("PID Error", "%.2f°", error);
        }
    }
    public void setInitialPose(Pose2D pose) {
        if (robotPose != null) {
            robotPose.setPose(pose);
            telemetry.addData("[System]", "Odometry Pose Overridden.");
        }
    }
}