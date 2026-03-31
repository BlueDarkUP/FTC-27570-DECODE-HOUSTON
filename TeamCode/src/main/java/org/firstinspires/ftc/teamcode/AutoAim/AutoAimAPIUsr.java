package org.firstinspires.ftc.teamcode.AutoAim;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name="AutoAimAPIUsr", group="Production")
public class AutoAimAPIUsr extends LinearOpMode {

    private AutoAimSubsystem autoAim;

    // ================= 云台硬件与 PID 参数 =================
    private DcMotorEx turretMotor;
    private final double kP = 0.035;
    private final double kI = 0.000;
    private final double kD = 0.00145;

    private double integralSum = 0;
    private double lastError = 0;
    private ElapsedTime timer = new ElapsedTime();
    // ===================================================

    final double TARGET_X_WORLD = 132.0;
    final double TARGET_Y_WORLD = 136.0;

    @Override
    public void runOpMode() {

        telemetry.addLine("Initializing Hardware, Please wait...");
        telemetry.update();

        // 1. 初始化云台电机
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        // 比赛中建议用 BRAKE（刹车）模式，能更稳地锁死目标
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 2. 初始化自动瞄准子系统 (内部包含了 Pinpoint)
        autoAim = new AutoAimSubsystem(hardwareMap, telemetry);

        // ================= 【核心修复点】 =================
        // 给 goBILDA Pinpoint 的内部 IMU 留出 0.5 秒的开机自动清零时间
        // 防止它的清零程序覆盖掉我们接下来要写入的坐标
        sleep(500);
        // ===============================================

        Pose2D startPose = new Pose2D(DistanceUnit.INCH, 72, 72, AngleUnit.DEGREES, 0);
        autoAim.setInitialPose(startPose);

        // 在按下 Play 键之前持续刷新，让你在手机上能稳定看到 72, 72
        while (opModeInInit()) {
            autoAim.update(TARGET_X_WORLD, TARGET_Y_WORLD);
            telemetry.addLine("Ready to Start - Odometry is Active!");
            telemetry.update();
        }

        waitForStart();
        timer.reset(); // 启动时重置时间

        while (opModeIsActive()) {

            // 呼叫 AutoAim 计算瞄准参数
            AutoAimSubsystem.TurretCommand aimCommand = autoAim.update(TARGET_X_WORLD, TARGET_Y_WORLD);

            if (aimCommand.hasTarget) {
                // =============== 执行云台 PID 控制 ===============

                // 子系统已经帮我们算好了最安全的误差角度
                double error = aimCommand.pidErrorDeg;

                double dt = timer.seconds();
                if (dt == 0) dt = 0.001; // 防止除以 0

                // PID 核心计算
                integralSum += error * dt;
                double derivative = (error - lastError) / dt;

                double power = (kP * error) + (kI * integralSum) + (kD * derivative);

                // 限制电机最大输出功率在 [-1.0, 1.0] 范围内
                power = Math.max(-1.0, Math.min(1.0, power));

                turretMotor.setPower(power);

                // 更新历史数据
                lastError = error;
                timer.reset();

                // 在屏幕上打印当前控制数据
                telemetry.addData("[Target] Locked", "X:%.0f Y:%.0f", TARGET_X_WORLD, TARGET_Y_WORLD);
                telemetry.addData("[Turret] Power", "%.2f (Err: %.1f°)", power, error);

                // ==================================================
                // (此处将来可以继续补充 ShooterMotor 和 PitchServo 的控制)
                // shooterMotor.setVelocity(aimCommand.targetRpm);
                // pitchServo.setPosition(aimCommand.targetPitch);

            } else {
                // 如果丢失目标，或者目标太近算法不输出时：电机彻底断电停转
                turretMotor.setPower(0);

                // 清理 PID 缓存，防止目标重新出现时云台抽搐 (Integral Windup)
                integralSum = 0;
                lastError = 0;
                timer.reset();

                telemetry.addLine("[Target] LOST or TOO CLOSE");
            }

            telemetry.update();
        }
    }
}