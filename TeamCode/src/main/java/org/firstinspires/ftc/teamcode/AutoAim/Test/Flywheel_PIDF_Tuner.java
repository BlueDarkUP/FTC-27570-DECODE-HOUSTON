package org.firstinspires.ftc.teamcode.AutoAim.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config // ！！关键：加上这个注解，Dashboard 就能识别并允许修改下面的 static 变量
@TeleOp(name = "Flywheel_Dashboard_Tuner", group = "Tuning")
public class Flywheel_PIDF_Tuner extends LinearOpMode {

    // ！！关键：必须是 public static，才能在 Dashboard 右侧的 Configuration 面板中实时修改
    public static double targetVelocity = 0.0; // 目标速度 (Ticks per second)
    public static double kF = 0.0;
    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;

    private DcMotorEx motorSH;
    private DcMotorEx motorHS;

    // PID 内部变量
    double integralSum = 0.0;
    double lastError = 0.0;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {

        // ！！关键：将默认的 telemetry 替换为 MultipleTelemetry，这样数据会同时发给手机和电脑网页
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // 1. 初始化硬件
        motorSH = hardwareMap.get(DcMotorEx.class, "SH");
        motorHS = hardwareMap.get(DcMotorEx.class, "HS");

        // 设置方向：SH 正向，HS 反向
        motorSH.setDirection(DcMotorSimple.Direction.FORWARD);
        motorHS.setDirection(DcMotorSimple.Direction.REVERSE);

        // 设置模式：使用我们自定义的 PID 计算，不使用自带 PID
        motorSH.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorHS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // 飞轮停止时设为滑行(FLOAT)，保护齿轮箱
        motorSH.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorHS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("等待开始...");
        telemetry.addLine("请在电脑浏览器打开 http://192.168.43.1:8080/dash (如果你用的是Control Hub)");
        telemetry.update();

        waitForStart();
        timer.reset();

        // 2. 运行阶段
        while (opModeIsActive()) {

            // --- 读取 SH 的编码器速度 ---
            double currentVelocity = motorSH.getVelocity();

            // --- 自定义 PIDF 计算 ---
            double dt = timer.seconds();
            timer.reset();

            // 防止 dt 为 0 导致除以 0 的错误
            if (dt == 0) dt = 0.001;

            double error = targetVelocity - currentVelocity;

            integralSum += error * dt;
            double derivative = (error - lastError) / dt;
            lastError = error;

            // 核心公式：Power = F + P + I + D
            double power = (targetVelocity * kF) + (error * kP) + (integralSum * kI) + (derivative * kD);

            // 限制输出在 -1.0 到 +1.0 之间
            power = Math.max(-1.0, Math.min(1.0, power));

            // 如果目标速度为 0，强制切断动力并清空积分，防止意外转动
            if (targetVelocity == 0) {
                power = 0;
                integralSum = 0;
            }

            // --- 同步输出动力 ---
            motorSH.setPower(power);
            motorHS.setPower(power);

            // --- 发送数据到 Dashboard 供画图使用 ---
            // 只要把数字 addData 进去，Dashboard 的 Graph 视图就能自动画出折线图
            telemetry.addData("1_Target_Velocity", targetVelocity);
            telemetry.addData("2_Current_Velocity", currentVelocity);
            telemetry.addData("3_Motor_Power", power);
            telemetry.update();
        }

        // 3. 结束时停止
        motorSH.setPower(0);
        motorHS.setPower(0);
    }
}