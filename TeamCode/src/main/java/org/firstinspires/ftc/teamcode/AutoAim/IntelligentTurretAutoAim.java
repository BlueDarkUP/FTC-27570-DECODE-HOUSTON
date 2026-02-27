package org.firstinspires.ftc.teamcode.AutoAim;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.vision.EchoLapse.PinpointPoseProvider;

@TeleOp(name="DEBUG: Ultimate Turret AutoAim - Sandbox", group="Production")
public class IntelligentTurretAutoAim extends LinearOpMode {

    // 1. 硬件声明
    private PinpointPoseProvider robotPose;
    private PinpointPoseProvider turretPose;
    private Limelight3A ll;
    private DcMotorEx turretEncoder;

    // 2. 物理参数与限位配置
    final double FIELD_OFFSET_X = 72.0;
    final double FIELD_OFFSET_Y = 72.0;
    final double TARGET_X_WORLD = 132.0;
    final double TARGET_Y_WORLD = 136.0;

    final double TURRET_OFFSET_FWD = -5.0;
    final double TURRET_OFFSET_LEFT = 0.0;
    final double TURRET_TICKS_PER_REV = 8192.0;

    final double TURRET_SOFT_LIMIT = 260.0;
    final double STATIONARY_SPEED_LIMIT = 2.0;

    // 3. 算法参数
    final double MAX_PHYSICAL_ACCEL = 300.0;
    final double IMPACT_COOLDOWN_MS = 300.0;
    final double ALPHA_NORMAL = 0.80;
    final double ALPHA_IMPACT = 0.05;

    private long lastLoopTime = 0;

    // [修复点 3]: 分离真实加速度计算和速度平滑预测的变量
    private double lastRawVxField = 0;
    private double lastRawVyField = 0;
    private double lastSmoothVxField = 0;
    private double lastSmoothVyField = 0;

    private boolean isImpactDetected = false;
    private ElapsedTime impactTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        // [修复点 4]: 强化硬件单设备隔离，缺失任何设备都不会崩溃
        try {
            robotPose = new PinpointPoseProvider(hardwareMap, "odo");
            robotPose.initialize();
            telemetry.addLine("[OK] Chassis Pinpoint Linked.");
        } catch (Exception e) {
            telemetry.addLine("[FATAL] Chassis ODO missing. Program cannot run.");
            telemetry.update();
            return; // 底盘里程计是唯一强依赖，没有直接退出
        }

        try {
            ll = hardwareMap.get(Limelight3A.class, "ll");
            ll.pipelineSwitch(0);
            ll.start();
            telemetry.addLine("[OK] Limelight Linked.");
        } catch (Exception e) {
            telemetry.addLine("[WARN] Limelight Missing. Running in Odom-Only Mode.");
        }

        try {
            turretPose = new PinpointPoseProvider(hardwareMap, "turretPinpoint");
            turretPose.initialize();
            turretEncoder = hardwareMap.get(DcMotorEx.class, "turretEncoder");
            telemetry.addLine("[OK] Turret Hardware Linked.");
        } catch (Exception e) {
            telemetry.addLine("[WARN] Turret hardware missing. Virtual Turret Mode Active.");
        }

        telemetry.update();
        waitForStart();
        lastLoopTime = System.nanoTime();

        while (opModeIsActive()) {
            robotPose.update();
            if (turretPose != null) turretPose.update();

            double rX = robotPose.getX(DistanceUnit.INCH);
            double rY = robotPose.getY(DistanceUnit.INCH);
            double rH_Rad = robotPose.getHeading(AngleUnit.RADIANS);
            double rVx = robotPose.getXVelocity(DistanceUnit.INCH);
            double rVy = robotPose.getYVelocity(DistanceUnit.INCH);
            double rOmega = robotPose.getHeadingVelocity(AngleUnit.RADIANS);

            double speed = Math.hypot(rVx, rVy);

            // [逻辑安全点]: Limelight 更新包裹在判空内
            if (ll != null) {
                LLResult result = ll.getLatestResult();
                if (result != null && result.isValid() && speed < STATIONARY_SPEED_LIMIT) {
                    Pose3D botpose = result.getBotpose();
                    double llX_Corner = (botpose.getPosition().x * 39.3701) + FIELD_OFFSET_X;
                    double llY_Corner = (botpose.getPosition().y * 39.3701) + FIELD_OFFSET_Y;
                    double llHeading = botpose.getOrientation().getYaw(AngleUnit.RADIANS);

                    double latency = (result.getCaptureLatency() + result.getTargetingLatency()) / 1000.0;
                    Pose2D correctedPose = new Pose2D(DistanceUnit.INCH,
                            llX_Corner + (rVx * latency),
                            llY_Corner + (rVy * latency),
                            AngleUnit.RADIANS, llHeading);
                    robotPose.setPose(correctedPose);
                }
            }

            // 完美运动学保留
            double cosH = Math.cos(rH_Rad);
            double sinH = Math.sin(rH_Rad);
            double turretX = rX + (TURRET_OFFSET_FWD * (-sinH)) + (TURRET_OFFSET_LEFT * (-cosH));
            double turretY = rY + (TURRET_OFFSET_FWD * ( cosH)) + (TURRET_OFFSET_LEFT * (-sinH));
            double tanVx = (TURRET_OFFSET_FWD * -cosH * rOmega) + (TURRET_OFFSET_LEFT * sinH * rOmega);
            double tanVy = (TURRET_OFFSET_FWD * -sinH * rOmega) + (TURRET_OFFSET_LEFT * -cosH * rOmega);
            double turretVx = rVx + tanVx;
            double turretVy = rVy + tanVy;

            // [修复点 3 核心]: 加速度计算必须使用 Raw 数据，否则会被滤波钝化
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

            // 仅对预测自瞄使用滤波，保证准星平滑
            double alpha = isImpactDetected ? ALPHA_IMPACT : ALPHA_NORMAL;
            double smoothVx = lastSmoothVxField * (1.0 - alpha) + turretVx * alpha;
            double smoothVy = lastSmoothVyField * (1.0 - alpha) + turretVy * alpha;
            lastSmoothVxField = smoothVx;
            lastSmoothVyField = smoothVy;
            lastLoopTime = System.nanoTime();

            AimCalculator.AimResult aim = AimCalculator.solveAim(
                    turretX, turretY, smoothVx, smoothVy, TARGET_X_WORLD, TARGET_Y_WORLD
            );

            double error = 0;
            if (aim != null) {
                double targetAbsHeading = aim.algYaw - 90.0;
                double currentChassisHeading = robotPose.getHeading(AngleUnit.DEGREES);

                // 安全读取当前云台机械位置 (解决空指针)
                double turretRelDeg = (turretEncoder != null) ? (turretEncoder.getCurrentPosition() / TURRET_TICKS_PER_REV * 360.0) : 0.0;

                // [修复点 2 核心]: 统一逻辑流，让撞击保护和正常模式都接受死区洗礼
                double currentGunAbsHeading;
                if (isImpactDetected && turretPose != null) {
                    // 如果发生撞击，依靠云台 IMU 获知当前炮管绝对朝向
                    currentGunAbsHeading = turretPose.getHeading(AngleUnit.DEGREES);
                } else {
                    // 正常情况，依靠底盘 IMU + 相对编码器 获知炮管绝对朝向
                    currentGunAbsHeading = currentChassisHeading + turretRelDeg;
                }

                // 1. 计算理论上我们需要向左/右转多少度（最短路径）
                double rawTurnDiff = AngleUnit.normalizeDegrees(targetAbsHeading - currentGunAbsHeading);

                // 2. 预测转完之后，编码器会停在什么位置
                double futurePosition = turretRelDeg + rawTurnDiff;

                // 3. 安全绕行逻辑 (Wrap-around防线缆缠绕)
                if (futurePosition > TURRET_SOFT_LIMIT) {
                    futurePosition -= 360.0;
                } else if (futurePosition < -TURRET_SOFT_LIMIT) {
                    futurePosition += 360.0;
                }

                // 4. 死区截断保护 (即使绕路也到达不了，强制卡在安全极限处)
                if (futurePosition > TURRET_SOFT_LIMIT) futurePosition = TURRET_SOFT_LIMIT;
                if (futurePosition < -TURRET_SOFT_LIMIT) futurePosition = -TURRET_SOFT_LIMIT;

                // 5. 最终输出给电机的安全误差
                error = futurePosition - turretRelDeg;
            }

            // ==========================================
            // 极高密度的仿真与测试 Telemetry 打印
            // ==========================================
            // [修复点 1]: 彻底解决空指针问题
            double safeTurretPos = (turretEncoder != null) ? (turretEncoder.getCurrentPosition() / TURRET_TICKS_PER_REV * 360.0) : 0.0;

            telemetry.addData("--- SYSTEM STATUS ---", isImpactDetected ? "[! IMPACT !]" : "[ OK ]");
            telemetry.addData("Turret Mechanical Pos", "%.1f°", safeTurretPos);
            telemetry.addData("Raw Accel", "%.1f in/s²", accel);

            telemetry.addLine("\n--- VIRTUAL KINEMATICS ---");
            telemetry.addData("Chassis Center", "X:%.1f  Y:%.1f", rX, rY);
            telemetry.addData("Virtual Turret Center", "X:%.1f  Y:%.1f", turretX, turretY);
            telemetry.addData("Chassis Speed", "Vx:%.1f  Vy:%.1f", rVx, rVy);
            telemetry.addData("Turret Swing Speed", "Vx:%.1f  Vy:%.1f", turretVx, turretVy);

            if(aim != null) {
                telemetry.addLine("\n--- PREDICTIVE AIM (Sandbox) ---");
                telemetry.addData("Target Pos", "(X:%.1f, Y:%.1f)", TARGET_X_WORLD, TARGET_Y_WORLD);
                telemetry.addData("Aim Pitch & RPM", "Pitch: %.1f | RPM: %.0f", aim.pitch, aim.rpm);
                telemetry.addData("Aim World Angle", "%.2f°", aim.algYaw);
                telemetry.addData("Predicted Flight Time", "%.2f s", aim.flightTime);
                telemetry.addData(">> FINAL PID ERROR", "%.2f°", error);
            }
            telemetry.update();
        }
    }
}