package org.firstinspires.ftc.teamcode.AutoAim.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "Flywheel Full Auto Tuner (FF + Coasting + V-Comp)", group = "Tuning")
public class FlywheelFullAutoTuner extends OpMode {

    private DcMotorEx motorSH;
    private DcMotorEx motorHS;
    private VoltageSensor battery;
    private HardwareMap hardwareMapRef;

    public static double TICKS_PER_REV = 28.0;

    // =====================================
    // 测试用例阶梯配置 (仅正向单向测试，保护飞轮机械结构)
    // =====================================
    public static double[] KV_TEST_POWERS = {0.3, 0.45, 0.6, 0.75, 0.9, 1.0};
    public static double[] KA_TEST_POWERS = {0.5, 0.7, 0.9, 1.0};
    public static double[] PB_TEST_POWERS = {1.0, 0.8, 0.6, 0.4}; // 用于测算自然减速/刹车摩擦

    // 飞轮加速需要更长的时间达到稳态
    public static int KV_SPIN_TIME_MS = 2000;
    public static int PB_SPIN_TIME_MS = 2500;

    // =====================================
    // 滤波与补偿算法参数
    // =====================================
    public static double VEL_FILTER_ALPHA = 0.85;
    public static double ACCEL_FILTER_ALPHA = 0.20;
    public static double NOMINAL_VOLTAGE = 12.9;     // 匹配你 FlywheelSubsystem 的基准电压

    private enum State {
        START,
        KS_RUN, KS_WAIT,
        KV_RUN, KV_WAIT,
        KA_RUN, KA_WAIT,
        PB_RUN, PB_COAST, PB_WAIT,
        CALCULATE, DONE
    }

    private State state = State.START;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime voltageTimer = new ElapsedTime();

    private long lastTime = 0;
    private double filteredVel = 0.0;
    private double lastFilteredVel = 0.0;
    private double filteredAccel = 0.0;
    private boolean isInitialized = false;

    // 电压监测
    private double currentVoltage = 12.9;
    private double sumVoltage = 0.0;
    private int voltageSamples = 0;

    // 运行态变量
    private int iteration = 0;
    private double currentTestPower = 0.0;
    private double pbStartCoastTicks = 0.0;
    private double pbMeasuredCoastVel = 0.0;

    // 数据样本集
    private final List<Double> ksSamples = new ArrayList<>();
    private final List<double[]> kvData = new ArrayList<>();
    private final List<double[]> kaData = new ArrayList<>();
    private final List<double[]> pbData = new ArrayList<>();

    // 最终系数
    private double rawKs = 0, rawKv = 0, rawKa = 0, rawKLin = 0, rawKQuad = 0;
    private double compKs = 0, compKv = 0, compKa = 0, compKLin = 0, compKQuad = 0;
    private double avgTestVoltage = 0;

    @Override
    public void init() {
        hardwareMapRef = hardwareMap;

        motorSH = hardwareMap.get(DcMotorEx.class, "SH");
        motorHS = hardwareMap.get(DcMotorEx.class, "HS");

        // 飞轮方向配置
        motorSH.setDirection(DcMotorSimple.Direction.FORWARD);
        motorHS.setDirection(DcMotorSimple.Direction.REVERSE);

        setMotorsMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        setMotorsMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // 刹车模式，测试其急停摩擦力
        motorSH.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorHS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Ready to auto-tune Flywheel (FF & Braking friction).");
        telemetry.addLine("WARNING: Keep clear of the Shooter. It will spool up automatically.");
        telemetry.update();
    }

    @Override
    public void start() {
        timer.reset();
        voltageTimer.reset();
        lastTime = System.nanoTime();
    }

    private void switchState(State newState) {
        state = newState;
        timer.reset();
    }

    private void setMotorsPower(double power) {
        motorSH.setPower(power);
        motorHS.setPower(power);
    }

    private void setMotorsMode(DcMotor.RunMode mode) {
        motorSH.setMode(mode);
        motorHS.setMode(mode);
    }

    private double getBatteryVoltage() {
        double maxVoltage = 0;
        for (VoltageSensor sensor : hardwareMapRef.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > maxVoltage) maxVoltage = v;
        }
        return Math.max(8.0, maxVoltage > 0 ? maxVoltage : NOMINAL_VOLTAGE);
    }

    @Override
    public void loop() {
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9;
        if (dt <= 0.00001) return;
        lastTime = currentTime;

        // 电压轮询
        if (voltageTimer.milliseconds() > 250 && state != State.DONE && state != State.CALCULATE) {
            currentVoltage = getBatteryVoltage();
            sumVoltage += currentVoltage;
            voltageSamples++;
            voltageTimer.reset();
        }

        // 飞轮测速：直接取用底层 TPS 信号，比差分绝对角度更精准
        double rawVel = motorSH.getVelocity();

        if (!isInitialized) {
            filteredVel = rawVel;
            lastFilteredVel = rawVel;
            filteredAccel = 0.0;
            isInitialized = true;
            return;
        }

        filteredVel = (VEL_FILTER_ALPHA * rawVel) + ((1.0 - VEL_FILTER_ALPHA) * filteredVel);

        double rawAccel = (filteredVel - lastFilteredVel) / dt;
        filteredAccel = (ACCEL_FILTER_ALPHA * rawAccel) + ((1.0 - ACCEL_FILTER_ALPHA) * filteredAccel);

        lastFilteredVel = filteredVel;

        switch (state) {
            case START:
                iteration = 0;
                ksSamples.clear();
                switchState(State.KS_RUN);
                break;

            // ================== kS 测算 (克服静摩擦) ==================
            case KS_RUN:
                if (iteration >= 2) {
                    double sum = 0;
                    for (Double val : ksSamples) sum += val;
                    rawKs = ksSamples.isEmpty() ? 0.0 : (sum / ksSamples.size());
                    iteration = 0;
                    switchState(State.KV_RUN);
                    break;
                }

                currentTestPower += 0.05 * dt;  // 缓慢推高电压
                setMotorsPower(currentTestPower);

                // TPS > 50 视为克服了静摩擦开始持续转动
                if (Math.abs(filteredVel) > 50.0) {
                    ksSamples.add(currentTestPower);
                    currentTestPower = 0.0;
                    setMotorsPower(0);
                    switchState(State.KS_WAIT);
                }
                break;

            case KS_WAIT:
                // 动态等待机制：直到飞轮完全停稳才进入下一次测试
                if (Math.abs(filteredVel) < 10.0 && timer.milliseconds() > 500) {
                    iteration++;
                    switchState(State.KS_RUN);
                }
                break;

            // ================== kV 测算 (速度前馈) ==================
            case KV_RUN:
                if (iteration >= KV_TEST_POWERS.length) { // 飞轮单向测试
                    iteration = 0;
                    rawKv = calculateLeastSquaresSlope(kvData);
                    switchState(State.KA_RUN);
                    break;
                }

                double kvTestPower = KV_TEST_POWERS[iteration];
                setMotorsPower(kvTestPower);

                // 等待 2 秒，让飞轮彻底达到极速稳态
                if (timer.milliseconds() > KV_SPIN_TIME_MS) {
                    if (Math.abs(filteredVel) > 100.0) {
                        double netPower = Math.max(0, kvTestPower - rawKs);
                        kvData.add(new double[]{Math.abs(filteredVel), netPower});
                    }
                    setMotorsPower(0);
                    switchState(State.KV_WAIT);
                }
                break;

            case KV_WAIT:
                if (Math.abs(filteredVel) < 10.0 && timer.milliseconds() > 500) {
                    iteration++;
                    switchState(State.KV_RUN);
                }
                break;

            // ================== kA 测算 (加速度前馈) ==================
            case KA_RUN:
                if (iteration >= KA_TEST_POWERS.length) {
                    iteration = 0;
                    switchState(State.PB_RUN);
                    break;
                }

                double kaTestPower = KA_TEST_POWERS[iteration];
                setMotorsPower(kaTestPower);

                // 飞轮加速瞬间抓取高频加速度（最初 300 毫秒）
                if (timer.milliseconds() < 300) {
                    if (Math.abs(filteredAccel) > 50.0) {
                        double netPower = Math.max(0, kaTestPower - rawKs - (rawKv * Math.abs(filteredVel)));
                        kaData.add(new double[]{Math.abs(filteredAccel), netPower});
                    }
                }

                if (timer.milliseconds() > 400) {
                    setMotorsPower(0);
                    switchState(State.KA_WAIT);
                }
                break;

            case KA_WAIT:
                if (Math.abs(filteredVel) < 10.0 && timer.milliseconds() > 500) {
                    iteration++;
                    switchState(State.KA_RUN);
                }
                break;

            // ================== PB/Coast 测算 (减速刹车摩擦) ==================
            case PB_RUN:
                if (iteration >= PB_TEST_POWERS.length) {
                    switchState(State.CALCULATE);
                    break;
                }

                double pbPower = PB_TEST_POWERS[iteration];
                setMotorsPower(pbPower);

                if (timer.milliseconds() >= PB_SPIN_TIME_MS) {
                    pbMeasuredCoastVel = Math.abs(filteredVel);
                    pbStartCoastTicks = motorSH.getCurrentPosition();

                    // 飞轮测试通常不给反转刹车，而是给 0 测试自带阻尼和电机刹车
                    setMotorsPower(0);
                    switchState(State.PB_COAST);
                }
                break;

            case PB_COAST:
                // 等待完全停止
                if (Math.abs(filteredVel) < 10.0 && timer.milliseconds() > 500) {
                    double coastTicks = Math.abs(motorSH.getCurrentPosition() - pbStartCoastTicks);
                    pbData.add(new double[]{pbMeasuredCoastVel, coastTicks});
                    switchState(State.PB_WAIT);
                }
                break;

            case PB_WAIT:
                iteration++;
                switchState(State.PB_RUN);
                break;

            // ================== 结算阶段 ==================
            case CALCULATE:
                avgTestVoltage = voltageSamples > 0 ? (sumVoltage / voltageSamples) : getBatteryVoltage();

                rawKa = calculateLeastSquaresSlope(kaData);
                if (rawKa < 0) rawKa = 0.0;

                calculatePbCoefficients();

                // 电压归一化补偿逻辑 (对应 FlywheelSubsystem 里的 12.9V 基准)
                double voltageScaleRatio = avgTestVoltage / NOMINAL_VOLTAGE;

                compKs = rawKs * voltageScaleRatio;
                compKv = rawKv * voltageScaleRatio;
                compKa = rawKa * voltageScaleRatio;
                compKLin = rawKLin * voltageScaleRatio;
                compKQuad = rawKQuad * voltageScaleRatio;

                switchState(State.DONE);
                break;

            case DONE:
                setMotorsPower(0);
                telemetry.addLine("=== FLYWHEEL TUNING COMPLETE ===");
                telemetry.addData("Data Points", String.format("kS:%d, kV:%d, kA:%d, PB:%d", ksSamples.size(), kvData.size(), kaData.size(), pbData.size()));
                telemetry.addData("Avg Test Voltage", String.format("%.2f V", avgTestVoltage));
                telemetry.addLine("--- Feedforward Configs ---");
                telemetry.addData("FLYWHEEL_kS", String.format("%.6f", compKs));
                telemetry.addData("FLYWHEEL_kV", String.format("%.8f", compKv)); // 飞轮 kV 极小，保留8位
                telemetry.addData("FLYWHEEL_kA", String.format("%.8f", compKa));
                telemetry.addLine("--- Deceleration Coast/Brake Configs ---");
                telemetry.addData("FLYWHEEL_kLinear", String.format("%.8f", compKLin));
                telemetry.addData("FLYWHEEL_kQuadratic", String.format("%.8f", compKQuad));
                break;
        }

        // 实时面板更新
        if (state != State.DONE) {
            telemetry.addData("Phase", state.toString().split("_")[0]);
            telemetry.addData("State", state.toString());
            telemetry.addData("Voltage (Live)", String.format("%.2f V", currentVoltage));
            telemetry.addData("Velocity (TPS)", String.format("%.1f", filteredVel));

            // 实时展示等效的 RPM 供观测
            telemetry.addData("Equiv RPM", String.format("%.0f", (filteredVel * 60.0) / TICKS_PER_REV));

            if (state.toString().contains("KV")) telemetry.addData("Prog", iteration + " / " + KV_TEST_POWERS.length);
            else if (state.toString().contains("KA")) telemetry.addData("Prog", iteration + " / " + KA_TEST_POWERS.length);
            else if (state.toString().contains("PB")) telemetry.addData("Prog", iteration + " / " + PB_TEST_POWERS.length);
        }
        telemetry.update();
    }

    private double calculateLeastSquaresSlope(List<double[]> data) {
        if (data.isEmpty()) return 0.0;
        double sumXY = 0, sumXX = 0;
        for (double[] point : data) {
            sumXY += point[0] * point[1];
            sumXX += point[0] * point[0];
        }
        return sumXX == 0 ? 0.0 : sumXY / sumXX;
    }

    private void calculatePbCoefficients() {
        double sumX = 0, sumY = 0, sumXY = 0, sumXX = 0;
        int n = 0;

        for (double[] point : pbData) {
            double v = point[0]; // Velocity in TPS
            double d = point[1]; // Distance in Ticks
            if (v < 100.0) continue;

            double x = v;
            double y = d / v;

            sumX += x;
            sumY += y;
            sumXY += x * y;
            sumXX += x * x;
            n++;
        }

        if (n > 1) {
            double denominator = (n * sumXX) - (sumX * sumX);
            rawKQuad = ((n * sumXY) - (sumX * sumY)) / denominator;
            rawKLin = ((sumY * sumXX) - (sumX * sumXY)) / denominator;

            if (rawKQuad < 0) rawKQuad = 0;
            if (rawKLin < 0) rawKLin = 0;
        }
    }
}