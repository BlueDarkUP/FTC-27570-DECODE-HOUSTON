package org.firstinspires.ftc.teamcode.AutoAim.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "Turret Full Auto Tuner (FF + PB + V-Comp)", group = "Tuning")
public class TurretFullAutoTuner extends OpMode {

    private DcMotorEx Turret;

    public static double TICKS_PER_REV = 32798.0; //TODO: 确认实际云台绕一圈编码器读数

    // =====================================
    // 调试器安全与逻辑参数
    // =====================================
    public static double MAX_SAFE_ANGLE = 150.0;     // 测试时的云台绝对限位
    public static double BRAKE_BUFFER_ANGLE = 35.0;  // 加大缓冲：距离极限多少度提前介入回中/刹车
    public static double RETURN_POWER = 0.3;         // 每次测试结束后，回中使用的力量
    public static int REST_TIME_MS = 500;            // 测试间隔休息时间

    // =====================================
    // 测试用例阶梯配置 (已更新为你的要求)
    // =====================================
    public static double[] KV_TEST_POWERS = {0.3, 0.45, 0.6, 0.75, 0.8, 0.9, 1.0};
    public static double[] KA_TEST_POWERS = {0.5, 0.7, 0.9, 1.0};
    public static double[] PB_TEST_POWERS = {1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3};
    public static double PB_BRAKING_POWER = 0.3;     // 刹车测试时的固定反向力量
    public static int PB_SPIN_TIME_MS = 3000;        // 刹车测试时加速的最大超时时间

    // =====================================
    // 滤波与补偿算法参数
    // =====================================
    public static double VEL_FILTER_ALPHA = 0.85;
    public static double ACCEL_FILTER_ALPHA = 0.20;
    public static double NOMINAL_VOLTAGE = 12.0;     // 归一化基准电压

    private enum State {
        START,
        KS_RUN, KS_RETURN, KS_WAIT,
        KV_RUN, KV_RETURN, KV_WAIT,
        KA_RUN, KA_RETURN, KA_WAIT,
        PB_RUN, PB_BRAKE, PB_RETURN, PB_WAIT,
        CALCULATE, DONE
    }

    private State state = State.START;
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime voltageTimer = new ElapsedTime();

    private long lastTime = 0;
    private double lastAngle = 0.0;
    private double filteredVel = 0.0;
    private double lastFilteredVel = 0.0;
    private double filteredAccel = 0.0;
    private boolean isInitialized = false;

    // 电压监测
    private double currentVoltage = 12.0;
    private double sumVoltage = 0.0;
    private int voltageSamples = 0;

    // 运行态变量
    private int iteration = 0;
    private int currentDirection = 1;
    private double currentTestPower = 0.0;
    private double pbStartBrakeAngle = 0.0;
    private double pbMeasuredBrakeVel = 0.0;

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
        Turret = hardwareMap.get(DcMotorEx.class, "Turret");
        Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        telemetry.addLine("Ready to auto-tune Turret (FF & PB).");
        telemetry.update();
    }

    @Override
    public void start() {
        timer.reset();
        voltageTimer.reset();
        lastTime = System.nanoTime();
    }

    /** 强制状态切换封装，确保计时器重置 */
    private void switchState(State newState) {
        state = newState;
        timer.reset();
    }

    /** 健壮的电池电压读取，防止获取到 0V 的 Dummy 传感器 */
    private double getBatteryVoltage() {
        double maxVoltage = 0;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double v = sensor.getVoltage();
            if (v > maxVoltage) {
                maxVoltage = v;
            }
        }
        return maxVoltage > 0 ? maxVoltage : 12.0;
    }

    @Override
    public void loop() {
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9;
        if (dt <= 0.00001) return;
        lastTime = currentTime;

        double currentTicks = Turret.getCurrentPosition();
        double currentAngle = (currentTicks / TICKS_PER_REV) * 360.0;

        // 电压轮询
        if (voltageTimer.milliseconds() > 250 && state != State.DONE && state != State.CALCULATE) {
            currentVoltage = getBatteryVoltage();
            sumVoltage += currentVoltage;
            voltageSamples++;
            voltageTimer.reset();
        }

        if (!isInitialized) {
            lastAngle = currentAngle;
            filteredVel = 0.0;
            lastFilteredVel = 0.0;
            filteredAccel = 0.0;
            isInitialized = true;
            return;
        }

        // 滤波
        double rawVel = (currentAngle - lastAngle) / dt;
        filteredVel = (VEL_FILTER_ALPHA * rawVel) + ((1.0 - VEL_FILTER_ALPHA) * filteredVel);

        double rawAccel = (filteredVel - lastFilteredVel) / dt;
        filteredAccel = (ACCEL_FILTER_ALPHA * rawAccel) + ((1.0 - ACCEL_FILTER_ALPHA) * filteredAccel);

        lastAngle = currentAngle;
        lastFilteredVel = filteredVel;

        double limitThreshold = MAX_SAFE_ANGLE - BRAKE_BUFFER_ANGLE;
        boolean hitSafetyLimit = (currentAngle >= limitThreshold && currentDirection == 1) ||
                (currentAngle <= -limitThreshold && currentDirection == -1);

        // 【关键修复1】全局物理极限保护：不再跳转到 DONE，而是跳转到 CALCULATE 结算已收集的数据！
        if (Math.abs(currentAngle) > (MAX_SAFE_ANGLE + 15) && !isReturningState(state) && state != State.DONE && state != State.CALCULATE) {
            Turret.setPower(0);
            telemetry.addLine("!!! DANGER: PAST MAX SAFE ANGLE !!! FORCING EARLY CALCULATION");
            switchState(State.CALCULATE);
        }

        switch (state) {
            case START:
                iteration = 0;
                ksSamples.clear();
                switchState(State.KS_RUN);
                break;

            // ================== kS 测算 ==================
            case KS_RUN:
                if (iteration >= 2) {
                    double sum = 0;
                    for (Double val : ksSamples) sum += val;
                    rawKs = ksSamples.isEmpty() ? 0.0 : (sum / ksSamples.size());
                    iteration = 0;
                    switchState(State.KV_RUN);
                    break;
                }

                currentDirection = (iteration % 2 == 0) ? 1 : -1;
                currentTestPower += 0.1 * dt;
                Turret.setPower(currentTestPower * currentDirection);

                if (Math.abs(filteredVel) > 2.0 || hitSafetyLimit) {
                    ksSamples.add(currentTestPower);
                    currentTestPower = 0.0;
                    Turret.setPower(0);
                    switchState(State.KS_RETURN);
                }
                break;

            case KS_RETURN:
                if (returnToCenter(currentAngle) || timer.milliseconds() > 3000) {
                    Turret.setPower(0);
                    switchState(State.KS_WAIT);
                }
                break;

            case KS_WAIT:
                if (timer.milliseconds() >= REST_TIME_MS) {
                    iteration++;
                    switchState(State.KS_RUN);
                }
                break;

            // ================== kV 测算 ==================
            case KV_RUN:
                if (iteration >= KV_TEST_POWERS.length * 2) {
                    iteration = 0;
                    rawKv = calculateLeastSquaresSlope(kvData);
                    switchState(State.KA_RUN);
                    break;
                }

                int kvPowerIndex = iteration / 2;
                currentDirection = (iteration % 2 == 0) ? 1 : -1;
                double kvTestPower = KV_TEST_POWERS[kvPowerIndex];

                Turret.setPower(kvTestPower * currentDirection);

                if (timer.milliseconds() > 700 || hitSafetyLimit) {
                    if (Math.abs(filteredVel) > 3.0) {
                        double netPower = Math.max(0, kvTestPower - rawKs);
                        kvData.add(new double[]{Math.abs(filteredVel), netPower});
                    }
                    Turret.setPower(0);
                    switchState(State.KV_RETURN);
                }
                break;

            case KV_RETURN:
                if (returnToCenter(currentAngle) || timer.milliseconds() > 3000) {
                    Turret.setPower(0);
                    switchState(State.KV_WAIT);
                }
                break;

            case KV_WAIT:
                if (timer.milliseconds() >= REST_TIME_MS) {
                    iteration++;
                    switchState(State.KV_RUN);
                }
                break;

            // ================== kA 测算 ==================
            case KA_RUN:
                if (iteration >= KA_TEST_POWERS.length * 2) {
                    iteration = 0;
                    switchState(State.PB_RUN);
                    break;
                }

                int kaPowerIndex = iteration / 2;
                currentDirection = (iteration % 2 == 0) ? 1 : -1;
                double kaTestPower = KA_TEST_POWERS[kaPowerIndex];

                Turret.setPower(kaTestPower * currentDirection);

                if (timer.milliseconds() < 300) {
                    if (Math.abs(filteredAccel) > 10.0) {
                        double netPower = Math.max(0, kaTestPower - rawKs - (rawKv * Math.abs(filteredVel)));
                        kaData.add(new double[]{Math.abs(filteredAccel), netPower});
                    }
                }

                if (timer.milliseconds() > 400 || hitSafetyLimit) {
                    Turret.setPower(0);
                    switchState(State.KA_RETURN);
                }
                break;

            case KA_RETURN:
                if (returnToCenter(currentAngle) || timer.milliseconds() > 3000) {
                    Turret.setPower(0);
                    switchState(State.KA_WAIT);
                }
                break;

            case KA_WAIT:
                if (timer.milliseconds() >= REST_TIME_MS) {
                    iteration++;
                    switchState(State.KA_RUN);
                }
                break;

            // ================== PB 测算 ==================
            case PB_RUN:
                if (iteration >= PB_TEST_POWERS.length * 2) {
                    switchState(State.CALCULATE);
                    break;
                }

                int pbPowerIndex = iteration / 2;
                currentDirection = (iteration % 2 == 0) ? 1 : -1;
                double pbPower = PB_TEST_POWERS[pbPowerIndex] * currentDirection;

                Turret.setPower(pbPower);

                if (timer.milliseconds() >= PB_SPIN_TIME_MS || hitSafetyLimit) {
                    pbMeasuredBrakeVel = filteredVel;
                    pbStartBrakeAngle = currentAngle;
                    Turret.setPower(-currentDirection * PB_BRAKING_POWER);
                    switchState(State.PB_BRAKE);
                }
                break;

            case PB_BRAKE:
                // 【关键修复2】加入延时判定 timer > 150，防止进入刹车态瞬间因为噪声直接判定刹车完成
                if ((Math.signum(filteredVel) != currentDirection || Math.abs(filteredVel) < 2.0) && timer.milliseconds() > 150
                        || timer.milliseconds() > 3000) {
                    Turret.setPower(0);
                    double brakingDistance = Math.abs(currentAngle - pbStartBrakeAngle);
                    double absVel = Math.abs(pbMeasuredBrakeVel);

                    pbData.add(new double[]{absVel, brakingDistance});
                    switchState(State.PB_RETURN);
                }
                break;

            case PB_RETURN:
                if (returnToCenter(currentAngle) || timer.milliseconds() > 3000) {
                    Turret.setPower(0);
                    switchState(State.PB_WAIT);
                }
                break;

            case PB_WAIT:
                if (timer.milliseconds() >= REST_TIME_MS) {
                    iteration++;
                    switchState(State.PB_RUN);
                }
                break;

            // ================== 结算阶段 ==================
            case CALCULATE:
                avgTestVoltage = voltageSamples > 0 ? (sumVoltage / voltageSamples) : getBatteryVoltage();

                rawKa = calculateLeastSquaresSlope(kaData);
                if (rawKa < 0) rawKa = 0.0;

                calculatePbCoefficients();

                double voltageScaleRatio = avgTestVoltage / NOMINAL_VOLTAGE;

                compKs = rawKs * voltageScaleRatio;
                compKv = rawKv * voltageScaleRatio;
                compKa = rawKa * voltageScaleRatio;
                compKLin = rawKLin * voltageScaleRatio;
                compKQuad = rawKQuad * voltageScaleRatio;

                switchState(State.DONE);
                break;

            case DONE:
                Turret.setPower(0);
                telemetry.addLine("=== ALL TUNING COMPLETE ===");
                telemetry.addData("Data Points", String.format("kS:%d, kV:%d, kA:%d, PB:%d", ksSamples.size(), kvData.size(), kaData.size(), pbData.size()));
                telemetry.addData("Avg Test Voltage", String.format("%.2f V", avgTestVoltage));
                telemetry.addLine("--- Feedforward Configs ---");
                telemetry.addData("TURRET_kS", String.format("%.6f", compKs));
                telemetry.addData("TURRET_kV", String.format("%.6f", compKv));
                telemetry.addData("TURRET_kA", String.format("%.6f", compKa));
                telemetry.addLine("--- Predictive Brake Configs ---");
                telemetry.addData("TURRET_kLinear", String.format("%.6f", compKLin));
                telemetry.addData("TURRET_kQuadratic", String.format("%.6f", compKQuad));
                break;
        }

        // 实时面板更新
        if (state != State.DONE) {
            telemetry.addData("Phase", state.toString().split("_")[0]);
            telemetry.addData("State", state.toString());
            telemetry.addData("Voltage (Live)", String.format("%.2f V", currentVoltage));
            telemetry.addData("Angle (deg)", String.format("%.1f", currentAngle));
            telemetry.addData("Velocity (deg/s)", String.format("%.1f", filteredVel));

            if (state.toString().contains("KV")) telemetry.addData("Prog", iteration + " / " + (KV_TEST_POWERS.length * 2));
            else if (state.toString().contains("KA")) telemetry.addData("Prog", iteration + " / " + (KA_TEST_POWERS.length * 2));
            else if (state.toString().contains("PB")) telemetry.addData("Prog", iteration + " / " + (PB_TEST_POWERS.length * 2));
        }
        telemetry.update();
    }

    // 【关键修复3】将 PB_BRAKE 加入到返回态判定中，允许其在刹车时小幅越过绝对安全线不被强制切断
    private boolean isReturningState(State s) {
        return s == State.KS_RETURN || s == State.KV_RETURN || s == State.KA_RETURN || s == State.PB_RETURN || s == State.PB_BRAKE;
    }

    private boolean returnToCenter(double currentAngle) {
        if (Math.abs(currentAngle) > 5.0) {
            Turret.setPower(-Math.signum(currentAngle) * RETURN_POWER);
            return false;
        } else {
            Turret.setPower(0);
            return true;
        }
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
            double v = point[0];
            double d = point[1];
            if (v < 5.0) continue;

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