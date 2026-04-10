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
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.AutoAim.AutoAimSubsystem;

@TeleOp(name = "AutoAim Integrated TeleOp", group = "Competition")
public class AutoAimTeleOp extends LinearOpMode {

    // ================= 硬件声明 =================
    private DcMotor lf = null;
    private DcMotor rf = null;
    private DcMotor lb = null;
    private DcMotor rb = null;

    private DcMotorEx motorSH;
    private DcMotorEx motorHS;
    private DcMotorEx motorIntake;

    private Servo bbb;
    private Servo LP;
    private Servo RP;

    private AutoAimSubsystem autoAim;

    // ================= 常量配置 =================
    private final double TARGET_X_WORLD = 132.0;
    private final double TARGET_Y_WORLD = 136.0;

    private final double IDLE_VELOCITY = 1000.0;
    private final double VELOCITY_TOLERANCE = 100.0;
    private final double kF = 0.0003;

    private final double STALL_CURRENT_AMPS = 2.7;
    private final double STALL_COOLDOWN_SEC = 0.5;

    private final double LP_UP = 1;
    private final double LP_DOWN = 0.4;
    private final double RP_UP = 0;
    private final double RP_DOWN = 0.5;

    // ================= 状态变量 =================
    private boolean isShootingMode = false;
    private boolean lastCircleState = false;

    private double intakeBrakeReleaseTime = 0.0;

    // 【新增】云台复位保护计时变量
    private double unwindReverseEndTime = 0.0;
    private boolean wasUnwinding = false;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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

        motorSH = hardwareMap.get(DcMotorEx.class, "SH");
        motorHS = hardwareMap.get(DcMotorEx.class, "HS");
        motorIntake = hardwareMap.get(DcMotorEx.class, "Intake");

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
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bbb.setPosition(0);

        telemetry.addLine("Initializing Hardware, Please wait...");
        telemetry.update();

        autoAim = new AutoAimSubsystem(hardwareMap, telemetry);
        sleep(500);

        Pose2D startPose = new Pose2D(DistanceUnit.INCH, 72.0, 72.0, AngleUnit.DEGREES, 0.0);
        autoAim.setInitialPose(startPose);

        while (opModeInInit()) {
            AutoAimSubsystem.TurretCommand aimCommand = autoAim.update(TARGET_X_WORLD, TARGET_Y_WORLD);
            if (aimCommand.hasTarget) {
                setPitchServos(aimCommand.targetPitch);
            }
            telemetry.addLine("Ready to Start - Odometry & Turret are Active!");
            telemetry.update();
        }

        while (opModeIsActive()) {

            // [一] 底盘控制
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            lf.setPower((y + x + rx) / denominator);
            lb.setPower((y - x + rx) / denominator);
            rf.setPower((y - x - rx) / denominator);
            rb.setPower((y + x - rx) / denominator);

            // [二] 自瞄系统更新
            AutoAimSubsystem.TurretCommand aimCommand = autoAim.update(TARGET_X_WORLD, TARGET_Y_WORLD);

            // [三] 俯仰舵机
            if (aimCommand.hasTarget) {
                setPitchServos(aimCommand.targetPitch);
            }

            // [四] 模式切换
            boolean currentCircleState = gamepad1.b;
            if (currentCircleState && !lastCircleState) {
                isShootingMode = !isShootingMode;
            }
            lastCircleState = currentCircleState;

            // [五] 目标分配
            double targetVelocity = IDLE_VELOCITY;
            if (isShootingMode) {
                bbb.setPosition(0.18);
                if (aimCommand.hasTarget) {
                    targetVelocity = aimCommand.targetRpm;
                }
            } else {
                bbb.setPosition(0);
            }

            // [六] 飞轮控制
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
            // [七] 智能 Intake 与云台防误甩保护逻辑
            // ==========================================
            double intakeCurrent = motorIntake.getCurrent(CurrentUnit.AMPS);

            if (isShootingMode) {

                // 【新增逻辑】判断云台是否正在因为触碰软限位而大角度掉头复位
                if (aimCommand.hasTarget && aimCommand.isUnwinding) {

                    // 边缘检测：记录开始掉头的时刻，给定 0.2秒反转时间
                    if (!wasUnwinding) {
                        unwindReverseEndTime = getRuntime() + 0.2;
                    }

                    if (getRuntime() < unwindReverseEndTime) {
                        motorIntake.setPower(-1.0); // 强制反转退弹 0.2秒
                        telemetry.addData("🔴 发射系统", "⚠️ 云台复位中: Intake 反转退弹!");
                    } else {
                        motorIntake.setPower(0.0);  // 退弹完毕，强制刹车防误射
                        telemetry.addData("🔴 发射系统", "⚠️ 云台复位中: Intake 刹车等待");
                    }
                    intakeBrakeReleaseTime = 0.0; // 清理原有堵转时间防止幽灵干涉

                } else {
                    // 正常发射逻辑
                    if (Math.abs(error) <= VELOCITY_TOLERANCE && aimCommand.hasTarget) {
                        motorIntake.setPower(1.0);
                        telemetry.addData("🔴 发射系统", "开火中 (FIRE!)");
                    } else {
                        motorIntake.setPower(0.0);
                        telemetry.addData("🔴 发射系统", "预热/调速中...");
                    }
                }

            } else {
                // 怠速模式堵转保护
                if (intakeCurrent >= STALL_CURRENT_AMPS) {
                    intakeBrakeReleaseTime = getRuntime() + STALL_COOLDOWN_SEC;
                }
                if (getRuntime() < intakeBrakeReleaseTime) {
                    motorIntake.setPower(0.0);
                    telemetry.addData("🟢 发射系统", "⚠️ INTAKE堵转保护触发！(刹车冷却中)");
                } else {
                    motorIntake.setPower(0.8);
                    telemetry.addData("🟢 发射系统", "怠速中 (Intake 常转收件)");
                }
            }

            // 【新增】更新掉头状态，供下一轮做边缘检测
            wasUnwinding = aimCommand.isUnwinding;

            // [八] 遥测输出
            telemetry.addData("当前模式", isShootingMode ? "[ 发射模式 ]" : "[ 怠速模式 ]");
            telemetry.addData("bbb 舵机位置", bbb.getPosition());
            if (aimCommand.hasTarget) {
                telemetry.addData("Pitch 舵机", "LP: %.3f | RP: %.3f", LP.getPosition(), RP.getPosition());
            }
            telemetry.addData("飞轮当前 RPM", currentVelocity);
            telemetry.addData("飞轮目标 RPM", targetVelocity);
            telemetry.addData("RPM 误差", error);
            telemetry.addData("当前飞轮动力分配", "%.2f", power);
            telemetry.addData("Intake 电流 (A)", "%.2f", intakeCurrent);
            telemetry.update();
        }

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