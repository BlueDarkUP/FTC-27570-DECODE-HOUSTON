package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit; // 【新增】导入电流单位
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.AutoAim.AutoAimSubsystem;

@TeleOp(name = "AutoAim Integrated TeleOp", group = "Competition")
public class AutoAimTeleOp extends LinearOpMode {

    // ================= 硬件声明 =================
    // 底盘电机
    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lb = null;
    private DcMotor rb = null;

    // 射击与供弹电机
    private DcMotorEx motorSH;
    private DcMotorEx motorHS;
    private DcMotorEx motorIntake;

    // 状态与瞄准舵机
    private Servo bbb; // 模式切换指示/联动舵机
    private Servo LP;  // 左俯仰舵机
    private Servo RP;  // 右俯仰舵机

    // 自瞄子系统
    private AutoAimSubsystem autoAim;

    // ================= 常量配置 =================
    // 目标世界坐标
    private final double TARGET_X_WORLD = 132.0;
    private final double TARGET_Y_WORLD = 136.0;

    // 飞轮与供弹参数
    private final double IDLE_VELOCITY = 1000.0;     // 怠速转速
    private final double VELOCITY_TOLERANCE = 100.0;
    private final double kF = 0.0003;

    private final double STALL_CURRENT_AMPS = 4.0;   // 堵转判定电流阈值
    private final double STALL_COOLDOWN_SEC = 0.5;   // 触发保护后的停机冷却时间（防止高频抽搐）

    // ======== 俯仰舵机物理限位 (基于测试得出) ========
    private final double LP_UP = 1;
    private final double LP_DOWN = 0.4;
    private final double RP_UP = 0;
    private final double RP_DOWN = 0.5;

    // ================= 状态变量 =================
    private boolean isShootingMode = false;  // 是否处于发射模式
    private boolean lastCircleState = false; // 记录上一次按键状态（用于Toggle边缘检测）

    private double intakeBrakeReleaseTime = 0.0;

    @Override
    public void runOpMode() {
        // ========== 新增：将手机和网页端的遥测合并 ==========
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // --------------------------------------------------
        // 1. 底盘电机初始化
        // --------------------------------------------------
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");

        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // --------------------------------------------------
        // 2. 射击机构初始化 (包括所有电机与舵机)
        // --------------------------------------------------
        motorSH = hardwareMap.get(DcMotorEx.class, "SH");
        motorHS = hardwareMap.get(DcMotorEx.class, "HS");
        motorIntake = hardwareMap.get(DcMotorEx.class, "Intake"); // 【修改】实例化为 DcMotorEx

        bbb = hardwareMap.get(Servo.class, "bbb");
        LP = hardwareMap.get(Servo.class, "LP");
        RP = hardwareMap.get(Servo.class, "RP");

        motorSH.setDirection(DcMotorSimple.Direction.FORWARD);
        motorHS.setDirection(DcMotorSimple.Direction.REVERSE);
        motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);

        motorSH.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorHS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorSH.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorHS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // 刹车防卡件

        // 初始化时给予舵机默认位置 (非发射状态归位)
        bbb.setPosition(1.0);

        // --------------------------------------------------
        // 3. 自瞄子系统初始化与初始位姿设定
        // --------------------------------------------------
        telemetry.addLine("Initializing Hardware, Please wait...");
        telemetry.update();

        autoAim = new AutoAimSubsystem(hardwareMap, telemetry);

        // 强制等待 500ms，确保 Pinpoint 芯片和通讯完全初始化
        sleep(500);

        // 设定车体在场地上的起始绝对坐标 (自动阶段结束后的停泊点)
        Pose2D startPose = new Pose2D(DistanceUnit.INCH, 72.0, 72.0, AngleUnit.DEGREES, 0.0);
        autoAim.setInitialPose(startPose);

        // --------------------------------------------------
        // 4. Init 循环（比赛开始前的预热预瞄阶段）
        // --------------------------------------------------
        while (opModeInInit()) {
            AutoAimSubsystem.TurretCommand aimCommand = autoAim.update(TARGET_X_WORLD, TARGET_Y_WORLD);
            if (aimCommand.hasTarget) {
                setPitchServos(aimCommand.targetPitch);
            }
            telemetry.addLine("Ready to Start - Odometry & Turret are Active!");
            telemetry.update();
        }


        // --------------------------------------------------
        // 5. 主循环 (TeleOp阶段)
        // --------------------------------------------------
        while (opModeIsActive()) {

            // ==========================================
            // [一] 底盘手动控制 (Mecanum Drive)
            // ==========================================
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            lf.setPower((y + x + rx) / denominator);
            lb.setPower((y - x + rx) / denominator);
            rf.setPower((y - x - rx) / denominator);
            rb.setPower((y + x - rx) / denominator);

            // ==========================================
            // [二] 自瞄系统后台更新
            // ==========================================
            AutoAimSubsystem.TurretCommand aimCommand = autoAim.update(TARGET_X_WORLD, TARGET_Y_WORLD);

            // ==========================================
            // [三] 俯仰双舵机实时控制 (Pitch)
            // ==========================================
            if (aimCommand.hasTarget) {
                setPitchServos(aimCommand.targetPitch);
            }

            // ==========================================
            // [四] 模式切换逻辑 (Toggle)
            // ==========================================
            boolean currentCircleState = gamepad1.b; // 对应 PlayStation 圆圈键 / Xbox B键
            if (currentCircleState && !lastCircleState) {
                isShootingMode = !isShootingMode; // 切换状态
            }
            lastCircleState = currentCircleState;

            // ==========================================
            // [五] 飞轮目标转速与 bbb 舵机位置分配
            // ==========================================
            double targetVelocity = IDLE_VELOCITY;

            if (isShootingMode) {
                bbb.setPosition(0.0);
                if (aimCommand.hasTarget) {
                    targetVelocity = aimCommand.targetRpm;
                } else {
                    targetVelocity = IDLE_VELOCITY;
                }
            } else {
                bbb.setPosition(1.0);
            }

            // ==========================================
            // [六] 飞轮 Bang-Bang + Kf 控制逻辑
            // ==========================================
            double currentVelocity = motorSH.getVelocity();
            double error = targetVelocity - currentVelocity;
            double power = 0.0;

            if (currentVelocity < targetVelocity - VELOCITY_TOLERANCE) {
                power = 1.0;
            } else if (currentVelocity > targetVelocity + VELOCITY_TOLERANCE) {
                power = 0.0;
            } else {
                power = targetVelocity * kF;
            }
            power = Math.max(0.0, Math.min(1.0, power));

            motorSH.setPower(power);
            motorHS.setPower(power);

            // ==========================================
            // [七] 智能 Intake (供弹与堵转保护) 逻辑
            // ==========================================
            // 【新增】实时获取 Intake 电机的电流
            double intakeCurrent = motorIntake.getCurrent(CurrentUnit.AMPS);

            if (isShootingMode) {
                // 发射模式：严格受转速误差控制
                if (Math.abs(error) <= VELOCITY_TOLERANCE && aimCommand.hasTarget) {
                    motorIntake.setPower(1.0); // 速度在 Bang-Bang 容差内且有目标，全速喂件
                    telemetry.addData("🔴 发射系统", "开火中 (FIRE!)");
                } else {
                    motorIntake.setPower(0.0); // 掉速、加速或滑行降速中，强制停止喂件防卡弹
                    telemetry.addData("🔴 发射系统", "预热/调速中...");
                }

                // 进入发射模式时，重置堵转定时器，防止切回怠速后处于幽灵保护状态
                intakeBrakeReleaseTime = 0.0;

            } else {
                // 怠速模式：只要不发射，Intake 始终保持运转，同时包含【堵转保护】

                // 检测是否达到堵转电流阈值（≥4A）
                if (intakeCurrent >= STALL_CURRENT_AMPS) {
                    // 刷新停机倒计时，要求往后至少停转 0.5秒
                    intakeBrakeReleaseTime = getRuntime() + STALL_COOLDOWN_SEC;
                }

                // 判断是否处于保护冷却期内
                if (getRuntime() < intakeBrakeReleaseTime) {
                    motorIntake.setPower(0.0); // 立刻刹车
                    telemetry.addData("🟢 发射系统", "⚠️ INTAKE堵转保护触发！(刹车冷却中)");
                } else {
                    motorIntake.setPower(0.8); // 正常收件
                    telemetry.addData("🟢 发射系统", "怠速中 (Intake 常转收件)");
                }
            }

            // ==========================================
            // [八] 遥测信息汇总
            // ==========================================
            telemetry.addData("当前模式", isShootingMode ? "[ 发射模式 ]" : "[ 怠速模式 ]");
            telemetry.addData("bbb 舵机位置", bbb.getPosition());
            if (aimCommand.hasTarget) {
                telemetry.addData("Pitch 舵机", "LP: %.3f | RP: %.3f", LP.getPosition(), RP.getPosition());
            }
            telemetry.addData("飞轮当前 RPM", currentVelocity);
            telemetry.addData("飞轮目标 RPM", targetVelocity);
            telemetry.addData("RPM 误差", error);
            telemetry.addData("当前飞轮动力分配", "%.2f", power);

            // 【新增】监控 Intake 电流
            telemetry.addData("Intake 电流 (A)", "%.2f", intakeCurrent);

            telemetry.update();
        }

        // ==========================================
        // 停止阶段清零保护
        // ==========================================
        motorSH.setPower(0);
        motorHS.setPower(0);
        motorIntake.setPower(0);
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
    }

    private void setPitchServos(double targetPitch) {
        double clampedLP = Math.max(LP_DOWN, Math.min(LP_UP, targetPitch));
        double proportion = (clampedLP - LP_DOWN) / (LP_UP - LP_DOWN);
        double calculatedRP = RP_DOWN + proportion * (RP_UP - RP_DOWN);
        calculatedRP = Math.max(0.0, Math.min(1.0, calculatedRP));

        LP.setPosition(clampedLP);
        RP.setPosition(calculatedRP);
    }
}