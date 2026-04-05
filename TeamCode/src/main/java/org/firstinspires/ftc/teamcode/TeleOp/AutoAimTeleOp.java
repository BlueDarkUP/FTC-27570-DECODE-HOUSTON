package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
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
    private DcMotor motorIntake;

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
    private final double VELOCITY_TOLERANCE = 100.0; // 容差范围：±100

    // PIDF 参数
    private final double kF = 0.0003;
    private final double kP = 0.01;
    private final double kI = 0.0;
    private final double kD = 0.000001;

    // ================= 状态变量 =================
    private boolean isShootingMode = false;  // 是否处于发射模式
    private boolean lastCircleState = false; // 记录上一次按键状态（用于Toggle边缘检测）

    private double integralSum = 0.0;
    private double lastError = 0.0;
    private ElapsedTime pidTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
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
        motorIntake = hardwareMap.get(DcMotor.class, "Intake");
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
            // 在点击 Start 前，持续更新 Odom 并让云台自动瞄准目标
            AutoAimSubsystem.TurretCommand aimCommand = autoAim.update(TARGET_X_WORLD, TARGET_Y_WORLD);

            // 俯仰舵机在 Init 阶段也跟随预瞄
            if (aimCommand.hasTarget) {
                double lpTarget = aimCommand.targetPitch;
                double rpRaw = 1.0 - lpTarget + 0.1;
                LP.setPosition(lpTarget);
                RP.setPosition(Math.max(0.0, Math.min(1.0, rpRaw)));
            }

            telemetry.addLine("Ready to Start - Odometry & Turret are Active!");
            telemetry.update();
        }

        // 比赛正式开始（按下 Start 键后执行到此处）
        // 启动时重置 PID 定时器，防止第一帧 dt 爆炸过大
        pidTimer.reset();

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
            // 无论何种模式，自瞄系统持续解算云台偏航和弹道俯仰
            AutoAimSubsystem.TurretCommand aimCommand = autoAim.update(TARGET_X_WORLD, TARGET_Y_WORLD);

            // ==========================================
            // [三] 俯仰双舵机实时控制 (Pitch)
            // ==========================================
            // 只要存在目标，就让炮管始终对准目标
            if (aimCommand.hasTarget) {
                double lpTarget = aimCommand.targetPitch;
                double rpRaw = 1.0 - lpTarget + 0.1;
                double rpTarget = Math.max(0.0, Math.min(1.0, rpRaw));

                LP.setPosition(lpTarget);
                RP.setPosition(rpTarget);
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
            double targetVelocity = IDLE_VELOCITY; // 默认怠速

            if (isShootingMode) {
                // 处于发射模式：bbb 舵机立刻打到 0.0
                bbb.setPosition(0.0);

                if (aimCommand.hasTarget) {
                    targetVelocity = aimCommand.targetRpm; // 如果有目标，使用自瞄解算的 RPM
                } else {
                    // 发射模式下但丢失目标，安全起见保持怠速
                    targetVelocity = IDLE_VELOCITY;
                }
            } else {
                // 不处于发射模式（怠速）：bbb 舵机复位到 1.0
                bbb.setPosition(1.0);
            }

            // ==========================================
            // [六] 飞轮 PIDF 闭环控制与自然减速逻辑
            // ==========================================
            double currentVelocity = motorSH.getVelocity();

            double dt = pidTimer.seconds();
            pidTimer.reset();
            if (dt == 0) dt = 0.001; // 防止除以 0

            double error = targetVelocity - currentVelocity;
            double power = 0.0;

            // 如果当前转速远超目标转速（例如切换回 Intake 怠速模式时），触发自然滑行
            if (error < -VELOCITY_TOLERANCE) {
                power = 0.0; // 切断输出，依靠初始化的 FLOAT 属性让飞轮由于阻力自然减速
                integralSum = 0.0; // 必须清空积分！防止在这段期间累积巨大的负误差，导致后续重加速无力
                lastError = error; // 仍然更新最后误差，保证下次进入 PID 时微分项不出错
            }
            // 否则（处于加速状态或转速维持在容差范围内），正常通过 PID 控制
            else {
                integralSum += error * dt;
                double derivative = (error - lastError) / dt;
                lastError = error;

                power = (targetVelocity * kF) + (error * kP) + (integralSum * kI) + (derivative * kD);
            }

            // 绝对禁止反转电流输出，飞轮的 power 必须限制在 0.0 到 1.0 之间
            power = Math.max(0.0, Math.min(1.0, power));

            motorSH.setPower(power);
            motorHS.setPower(power);

            // ==========================================
            // [七] 智能 Intake (供弹) 逻辑
            // ==========================================
            if (isShootingMode) {
                // 发射模式：严格受转速误差控制
                if (Math.abs(error) <= VELOCITY_TOLERANCE && aimCommand.hasTarget) {
                    motorIntake.setPower(1.0); // 速度达标且有目标，全速喂件
                    telemetry.addData("🔴 发射系统", "开火中 (FIRE!)");
                } else {
                    motorIntake.setPower(0.0); // 掉速或加速中，强制停止喂件防卡弹
                    telemetry.addData("🔴 发射系统", "预热/调速中...");
                }
            } else {
                // 怠速模式：只要不发射，Intake 始终保持 0.8 的动力运转收件
                motorIntake.setPower(0.8);
                telemetry.addData("🟢 发射系统", "怠速中 (Intake 常转收件)");
            }

            // ==========================================
            // [八] 遥测信息汇总
            // ==========================================
            telemetry.addData("当前模式", isShootingMode ? "[ 发射模式 ]" : "[ 怠速模式 ]");
            telemetry.addData("bbb 舵机位置", bbb.getPosition());
            if (aimCommand.hasTarget) {
                telemetry.addData("Pitch 舵机", "LP: %.2f | RP: %.2f", LP.getPosition(), RP.getPosition());
            }
            telemetry.addData("飞轮当前 RPM", currentVelocity);
            telemetry.addData("飞轮目标 RPM", targetVelocity);
            telemetry.addData("RPM 误差", error);
            telemetry.addData("当前飞轮动力分配", "%.2f", power); // 新增遥测便于监控自然滑行状态

            // autoAim 内部的遥测已经通过 update() 传入，只需统一下发
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
}