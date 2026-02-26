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

@TeleOp(name="DEBUG: Ultimate Turret AutoAim (PPP) - Safety", group="Production")
public class IntelligentTurretAutoAim extends LinearOpMode {

    // ==========================================
    // 1. 硬件声明
    // ==========================================
    private PinpointPoseProvider robotPose;
    private PinpointPoseProvider turretPose;
    private Limelight3A ll;
    private DcMotorEx turretEncoder;

    // ==========================================
    // 2. 物理参数与限位配置
    // ==========================================
    final double FIELD_OFFSET_X = 72.0;
    final double FIELD_OFFSET_Y = 72.0;
    final double TARGET_X_WORLD = 132.0;
    final double TARGET_Y_WORLD = 136.0;

    final double TURRET_OFFSET_FWD = -5.0;
    final double TURRET_OFFSET_LEFT = 0.0;
    final double TURRET_TICKS_PER_REV = 8192.0;

    // --- 限位安全参数 ---
    final double TURRET_SOFT_LIMIT = 260.0; // 软限位（硬限位270度，预留10度缓冲）
    final double STATIONARY_SPEED_LIMIT = 2.0; // 英寸/秒，视为静止的视觉校准阈值

    // ==========================================
    // 3. 算法参数
    // ==========================================
    final double MAX_PHYSICAL_ACCEL = 300.0;
    final double IMPACT_COOLDOWN_MS = 300.0;
    final double ALPHA_NORMAL = 0.80;
    final double ALPHA_IMPACT = 0.05;

    private long lastLoopTime = 0;
    private double lastVxField = 0;
    private double lastVyField = 0;
    private boolean isImpactDetected = false;
    private ElapsedTime impactTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        robotPose = new PinpointPoseProvider(hardwareMap, "odo");
        robotPose.initialize();
        ll = hardwareMap.get(Limelight3A.class, "ll");
        ll.pipelineSwitch(0);
        ll.start();
        try {
            turretPose = new PinpointPoseProvider(hardwareMap, "turretPinpoint");
            turretPose.initialize();
            turretEncoder = hardwareMap.get(DcMotorEx.class, "turretEncoder");
        } catch (Exception e) { telemetry.addLine("Turret sensors missing"); }

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

            // [逻辑改动 1]：仅在低速且视觉置信度高时进行 Pose 重置
            LLResult result = ll.getLatestResult();
            if (result != null && result.isValid() && speed < STATIONARY_SPEED_LIMIT) {
                Pose3D botpose = result.getBotpose();
                double llX_Corner = (botpose.getPosition().x * 39.3701) + FIELD_OFFSET_X;
                double llY_Corner = (botpose.getPosition().y * 39.3701) + FIELD_OFFSET_Y;
                double llHeading = botpose.getOrientation().getYaw(AngleUnit.RADIANS);

                // 延迟补偿
                double latency = (result.getCaptureLatency() + result.getTargetingLatency()) / 1000.0;
                Pose2D correctedPose = new Pose2D(DistanceUnit.INCH,
                        llX_Corner + (rVx * latency),
                        llY_Corner + (rVy * latency),
                        AngleUnit.RADIANS, llHeading);

                robotPose.setPose(correctedPose); // 仅在低速执行，防止动态跳变干扰
            }

            // [步骤 2] 运动学解算 (云台场地坐标及速度)
            double cosH = Math.cos(rH_Rad);
            double sinH = Math.sin(rH_Rad);
            double turretX = rX + (TURRET_OFFSET_FWD * (-sinH)) + (TURRET_OFFSET_LEFT * (-cosH));
            double turretY = rY + (TURRET_OFFSET_FWD * ( cosH)) + (TURRET_OFFSET_LEFT * (-sinH));
            double tanVx = (TURRET_OFFSET_FWD * -cosH * rOmega) + (TURRET_OFFSET_LEFT * sinH * rOmega);
            double tanVy = (TURRET_OFFSET_FWD * -sinH * rOmega) + (TURRET_OFFSET_LEFT * -cosH * rOmega);
            double turretVx = rVx + tanVx;
            double turretVy = rVy + tanVy;

            // [步骤 3] 滤波与撞击检测
            double dt = (System.nanoTime() - lastLoopTime) / 1.0E9;
            if(dt < 0.001) dt = 0.001;
            double accel = Math.hypot((turretVx - lastVxField)/dt, (turretVy - lastVyField)/dt);
            if (accel > MAX_PHYSICAL_ACCEL) {
                isImpactDetected = true;
                impactTimer.reset();
            } else if (impactTimer.milliseconds() > IMPACT_COOLDOWN_MS) {
                isImpactDetected = false;
            }
            double alpha = isImpactDetected ? ALPHA_IMPACT : ALPHA_NORMAL;
            double smoothVx = lastVxField * (1.0 - alpha) + turretVx * alpha;
            double smoothVy = lastVyField * (1.0 - alpha) + turretVy * alpha;
            lastVxField = smoothVx; lastVyField = smoothVy; lastLoopTime = System.nanoTime();

            // [逻辑改动 2]：调用带有 2 次迭代补偿的自瞄解算器
            AimCalculator.AimResult aim = AimCalculator.solveAim(
                    turretX, turretY, smoothVx, smoothVy, TARGET_X_WORLD, TARGET_Y_WORLD
            );

            // [逻辑改动 3]：智能绕行与限位控制
            double error = 0;
            if (aim != null) {
                // 将数学角(0=Right)转为导航角(0=Forward)
                double targetAbsHeading = aim.algYaw - 90.0;
                double currentChassisHeading = robotPose.getHeading(AngleUnit.DEGREES);
                double turretRelDeg = (turretEncoder != null) ? (turretEncoder.getCurrentPosition() / TURRET_TICKS_PER_REV * 360.0) : 0;

                // 1. 计算理论上的目标相对角度 (距离底盘 0 度的位置)
                double targetRel = AngleUnit.normalizeDegrees(targetAbsHeading - currentChassisHeading);

                // 2. 计算基于当前云台位置的最短路径增量
                double shortestDiff = AngleUnit.normalizeDegrees(targetRel - turretRelDeg);

                // 3. 安全绕行逻辑：判断最短路径是否会导致越过极限
                double futurePosition = turretRelDeg + shortestDiff;

                if (futurePosition > TURRET_SOFT_LIMIT) {
                    // 如果向正向转会撞限位，强制反向转一整圈（绕远路）
                    shortestDiff -= 360;
                } else if (futurePosition < -TURRET_SOFT_LIMIT) {
                    // 如果向负旋转会撞限位，强制正向转一整圈
                    shortestDiff += 360;
                }

                // 4. 最终截断保护：如果计算后仍越位（目标点本身在死区内）
                double finalTargetPos = turretRelDeg + shortestDiff;
                if (finalTargetPos > TURRET_SOFT_LIMIT) shortestDiff = TURRET_SOFT_LIMIT - turretRelDeg;
                if (finalTargetPos < -TURRET_SOFT_LIMIT) shortestDiff = -TURRET_SOFT_LIMIT - turretRelDeg;

                // 5. 特殊模式：撞击时使用云台本地 IMU（如果可用）
                if (isImpactDetected && turretPose != null) {
                    double turretAbsImu = turretPose.getHeading(AngleUnit.DEGREES);
                    error = AngleUnit.normalizeDegrees(targetAbsHeading - turretAbsImu);
                } else {
                    error = shortestDiff;
                }

                // 此处添加电机控制逻辑：turretMotor.setPower(PID(error));
            }

            telemetry.addData("Turret Pos", "%.1f", (turretEncoder.getCurrentPosition() / TURRET_TICKS_PER_REV * 360.0));
            telemetry.addData("Aim Error", "%.2f", error);
            telemetry.addData("Status", isImpactDetected ? "IMPACT!" : "OK");
            telemetry.update();
        }
    }
}
