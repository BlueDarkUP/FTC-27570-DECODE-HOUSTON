package org.firstinspires.ftc.teamcode.AutoAim.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "Shooter PIDF 调优程序", group = "Debug")
public class ShooterTuning extends LinearOpMode {

    public static double TARGET_RPM = 0.0;
    public static double P = 0.0016;
    public static double I = 0.0;
    public static double D = 0.000;

    // 前馈与物理模型参数
    public static double kV = 0.00033290;
    public static double kS = 0.097208;

    // 新增：不对称限幅参数，限制最大反转（刹车）功率，保护同步带和齿轮箱
    // 默认设置为 -0.25 (即允许最多 25% 的反向拉力)
    public static double MAX_BRAKE_POWER = 0;

    public static double TICKS_PER_REV = 28.0;

    private DcMotorEx SH, HS;
    private VoltageSensor batteryVoltageSensor;
    private ElapsedTime timer = new ElapsedTime();

    private double lastError = 0;
    private double integralSum = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        SH = hardwareMap.get(DcMotorEx.class, "SH");
        HS = hardwareMap.get(DcMotorEx.class, "HS");

        // 初始化电压传感器，用于 KSKV 模型的实时电压补偿
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        SH.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        HS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        SH.setDirection(DcMotorSimple.Direction.FORWARD);
        HS.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "等待开始...");
        telemetry.update();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            double currentVelTPS = SH.getVelocity();
            double targetVelTPS = (TARGET_RPM * TICKS_PER_REV) / 60.0;

            double dt = timer.seconds();
            timer.reset();
            if (dt == 0) dt = 1e-9;

            double error = targetVelTPS - currentVelTPS;

            if (TARGET_RPM > 100) {
                integralSum += error * dt;
            } else {
                integralSum = 0;
            }

            // 防止积分饱和与除零异常
            double maxIntegral = 0.25;
            if (I != 0.0) {
                if (integralSum > maxIntegral / I) integralSum = maxIntegral / I;
                if (integralSum < -maxIntegral / I) integralSum = -maxIntegral / I;
            } else {
                integralSum = 0;
            }

            double derivative = (error - lastError) / dt;
            lastError = error;

            // 获取当前实时电池电压
            double currentVoltage = batteryVoltageSensor.getVoltage();

            // 计算前馈 (Feedforward)
            double feedforward = 0.0;
            if (targetVelTPS > 0) {
                feedforward = kS * Math.signum(targetVelTPS) + kV * targetVelTPS;
            }

            // 反馈部分 (PID)
            double pid = (P * error) + (I * integralSum) + (D * derivative);

            // 基础电机输出 (基于 12.5V 标称电压计算出的理想 power)
            double basePower = feedforward + pid;

            // 结合电压补偿动态修正输出
            double power = basePower * (12.5 / currentVoltage);

            if (TARGET_RPM <= 0) {
                power = 0;
                integralSum = 0;
            }

            // 修改核心：不对称限幅
            // 正向允许最大 1.0 (100%)，反向刹车限制在 MAX_BRAKE_POWER (如 -0.25)
            power = Math.max(MAX_BRAKE_POWER, Math.min(1.0, power));

            SH.setPower(power);
            HS.setPower(power);

            double currentRPM = (currentVelTPS * 60.0) / TICKS_PER_REV;
            telemetry.addData("0-Target RPM", TARGET_RPM);
            telemetry.addData("1-Actual RPM", currentRPM);
            telemetry.addData("2-Error RPM", TARGET_RPM - currentRPM);
            telemetry.addData("3-Motor Power", power);
            telemetry.addData("4-Integral Sum", integralSum);
            telemetry.addData("5-Current Voltage", currentVoltage);
            telemetry.update();
        }
    }
}