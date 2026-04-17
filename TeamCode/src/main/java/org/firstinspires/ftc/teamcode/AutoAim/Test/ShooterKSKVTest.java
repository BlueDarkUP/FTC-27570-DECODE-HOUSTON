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

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "Shooter KSKV 自动标定程序", group = "Debug")
public class ShooterKSKVTest extends LinearOpMode {

    // 测算 KV 时使用的测试功率阶梯，可以在 Dashboard 中修改
    public static double[] KV_POWERS = {0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};

    // 静摩擦力测试的判定阈值 (TPS)
    public static double KS_VELOCITY_THRESHOLD = 50.0;

    private DcMotorEx SH, HS;
    private VoltageSensor battery;

    private double kS = 0;
    private double kV = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // 1. 初始化 Dashboard 遥测
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // 2. 硬件映射 (完全参考你的 ShooterTuning 配置)
        SH = hardwareMap.get(DcMotorEx.class, "SH");
        HS = hardwareMap.get(DcMotorEx.class, "HS");
        battery = hardwareMap.voltageSensor.iterator().next();

        SH.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        HS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        SH.setDirection(DcMotorSimple.Direction.FORWARD);
        HS.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "初始化完成。请确保发射器周围清空，按下 Start 开始自动标定。");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            telemetry.addData("Status", "正在测算 kS (静摩擦力)...");
            telemetry.update();
            kS = calcKS();

            telemetry.addData("Status", "kS 测算完毕，正在测算 kV...");
            telemetry.update();
            kV = calcKV();

            // 标定完成，持续显示结果供你记录
            while (opModeIsActive()) {
                telemetry.addData("Status", "标定完成！请将以下值填入你的 ShooterTuning 程序中");
                telemetry.addData(">> 最终 kS", kS);
                telemetry.addData(">> 最终 kV", kV);
                telemetry.addData("测试结束时电压", battery.getVoltage());
                telemetry.update();
                idle();
            }
        }
    }

    /**
     * 测算 kS (克服静摩擦所需的最小功率)
     */
    private double calcKS() {
        double power = 0;
        int stable = 0;

        while (opModeIsActive()) {
            power += 0.005; // 每次极少量地增加功率
            setMotorsPower(power);
            sleep(200); // 等待电机响应

            // 必须使用 TPS (Ticks Per Second) 作为单位，以匹配你的主程序
            double currentVelocityTPS = SH.getVelocity();

            if (currentVelocityTPS > KS_VELOCITY_THRESHOLD) {
                stable++;
            } else {
                stable = 0;
            }

            telemetry.addData("当前测试环节", "寻找启动功率 (kS)");
            telemetry.addData("当前输出 Power", power);
            telemetry.addData("当前速度 TPS", currentVelocityTPS);
            telemetry.update();

            // 如果速度超过阈值并稳定，说明已经克服静摩擦
            if (stable >= 1) break;
        }

        setMotorsPower(0);
        sleep(500); // 停转缓冲

        double voltage = battery.getVoltage();
        // 将测得的实际功率归一化到 12.5V 标称电压下
        double normalizedKS = power * (voltage / 12.5);
        sleep(1000);

        return normalizedKS;
    }

    /**
     * 测算 kV (维持目标速度所需的功率比例)
     */
    private double calcKV() {
        List<Double> velocities = new ArrayList<>();
        List<Double> powers = new ArrayList<>();

        for (double p : KV_POWERS) {
            if (!opModeIsActive()) break;

            setMotorsPower(p);
            sleep(1000); // 给系统 1 秒钟加速并达到稳定速度

            double currentVelocityTPS = SH.getVelocity();
            velocities.add(currentVelocityTPS);
            powers.add(p);

            telemetry.addData("当前测试环节", "测算速度与功率关系 (kV)");
            telemetry.addData("目标测试 Power", p);
            telemetry.addData("测得稳定速度 TPS", currentVelocityTPS);
            telemetry.update();

            sleep(500);
        }

        setMotorsPower(0); // 测试完毕，关闭电机

        // 使用最小二乘法进行线性回归估算 kV
        // 公式：kV = sum( (Power - kS) * Velocity ) / sum( Velocity^2 )
        double num = 0;
        double den = 0;

        for (int i = 0; i < velocities.size(); i++) {
            double v = velocities.get(i);
            double p = powers.get(i) - kS; // 刨去静摩擦力占用的部分

            num += p * v;
            den += v * v;
        }

        double voltage = battery.getVoltage();
        // 归一化到 12.5V 标称电压下
        double normalizedKV = (num / den) * (voltage / 12.5);
        sleep(1000);

        return normalizedKV;
    }

    /**
     * 统一控制两个电机的辅助方法
     */
    private void setMotorsPower(double power) {
        SH.setPower(power);
        HS.setPower(power);
    }
}