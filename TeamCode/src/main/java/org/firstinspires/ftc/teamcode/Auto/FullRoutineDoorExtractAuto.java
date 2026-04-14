package org.firstinspires.ftc.teamcode.Auto;

import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

// 导入您专属的 Constants
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Full Routine with Door Extract Loop", group = "Autonomous")
public class FullRoutineDoorExtractAuto extends OpMode {

    // ==========================================================
    // === 1. 硬件与子系统变量 ===
    // ==========================================================
    private Follower follower;
    private DcMotorEx intakeMotor;
    private DcMotorEx turretMotor;

    private DcMotorEx motorSH;
    private DcMotorEx motorHS;
    private Servo bbb; // 开火挡板舵机

    private Servo LP;  // 俯仰舵机 LP
    private Servo RP;  // 俯仰舵机 RP

    // ==========================================================
    // === 2. 计时器与状态机变量 ===
    // ==========================================================
    private Timer pathTimer;
    private Timer opmodeTimer;
    private Timer cycleTimer;
    private int pathState;

    // ==========================================================
    // === 3. 云台双段 PIDF 参数与状态变量 ===
    // ==========================================================
    private final double STAGE_THRESHOLD = 14.0;
    private final double kP_far = 0.05;
    private final double kI_far = 0.00;
    private final double kD_far = 0.0003;

    private final double kP_near = 0.0000002;
    private final double kI_near = 0.0001;
    private final double kD_near = 0.0015;

    private final double kS_near = 0.28;
    private final double I_ZONE_near = 3.0;
    private final double MAX_INTEGRAL_POWER = 0.08;

    private final double ERROR_TOLERANCE = 1.5;
    private final double TURRET_TICKS_PER_REV = 32798;
    private final double TICKS_PER_DEGREE = TURRET_TICKS_PER_REV / 360.0;

    private double integralSum = 0;
    private double lastError = 0;
    private boolean wasInStage1 = false;
    private ElapsedTime pidTimer;

    // 全局云台目标角度
    private double turretTargetAngle = 0.0;

    // ==========================================================
    // === 4. 飞轮 PIDF + Bang-Bang 参数与状态变量 ===
    // ==========================================================
    private final double FW_RPM_LOWER_BOUND = 3000.0;
    private final double FW_RPM_UPPER_BOUND = 5050.0;
    private final double FW_MAX_TOLERANCE = 1000;
    private final double FW_MIN_TOLERANCE = 1000;
    private final double FW_SPOOL_UP_TOLERANCE = 100.0;

    private final double FW_kP = 0.011;
    private final double FW_kI = 0.0004;
    private final double FW_kD = 0.00000023;
    private final double FW_kF = 0.00033;
    private final double FW_TICKS_PER_REV = 28.0;

    private double flywheelIntegralSum = 0;
    private double flywheelLastErrorTPS = 0;
    private boolean isFlywheelReady = false;
    private ElapsedTime flywheelTimer;

    // 全局飞轮变量
    private double flywheelTargetRPM = 0.0;
    private double flywheelCurrentRPM = 0.0;

    // ==========================================================
    // === 5. 射击控制状态机变量 (Non-blocking) ===
    // ==========================================================
    private enum ShootMode { NONE, PRECISION, BLIND }
    private ShootMode currentShootMode = ShootMode.NONE;
    private ElapsedTime shootTimer;
    private double activeShootDuration = 0.0;

    // 俯仰舵机范围常数
    private final double LP_UP = 1;
    private final double LP_DOWN = 0.4;
    private final double RP_UP = 0;
    private final double RP_DOWN = 0.5;

    // ==========================================================
    // === 6. 新版路径声明区 (PathChains) ===
    // ==========================================================
    public PathChain diyigepaoda;
    public PathChain xidierpai;
    public PathChain fashedierpai;
    public PathChain kaimenzuo;
    public PathChain fashekaimenzuo;
    public PathChain zhunbeixidiyipai;
    public PathChain xidiyipai;
    public PathChain fashediyipai;
    public PathChain zhunbeixidisanpai;
    public PathChain xidisanpai;
    public PathChain fashedisanpai;
    public PathChain tingkao;

    private final Pose startPose = new Pose(35.000, 134.600, Math.toRadians(180));

    public void buildPaths() {
        diyigepaoda = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(35.000, 134.600),
                                new Pose(60.000, 59.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        xidierpai = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(60.000, 59.000),
                                new Pose(11.000, 59.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        fashedierpai = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(11.000, 59.000),
                                new Pose(45.000, 60.000),
                                new Pose(60.000, 83.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(226))
                .build();

        kaimenzuo = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.000, 83.000),
                                new Pose(45.000, 60.000),
                                new Pose(12.5, 59)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(226), Math.toRadians(163))
                .build();

        fashekaimenzuo = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(12.5, 59),
                                new Pose(35.000, 47.000),
                                new Pose(60.000, 83.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(163), Math.toRadians(226))
                .build();

        zhunbeixidiyipai = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(60.000, 83.000),
                                new Pose(45.000, 83.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(226), Math.toRadians(180))
                .setNoDeceleration()
                .build();

        xidiyipai = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(45.000, 83.000),
                                new Pose(19.000, 83.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        fashediyipai = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(19.000, 83.000),
                                new Pose(60.000, 83.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(230))
                .build();

        zhunbeixidisanpai = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(60.000, 83.000),
                                new Pose(45.000, 36.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(180))
                .setNoDeceleration()
                .build();

        xidisanpai = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(45.000, 36.000),
                                new Pose(11.000, 36.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        fashedisanpai = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(11.000, 36.000),
                                new Pose(47.308, 32.731),
                                new Pose(60.000, 83.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(250))
                .build();

        tingkao = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(60.000, 83.000),
                                new Pose(60.000, 56.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(250), Math.toRadians(90))
                .build();
    }

    // ==========================================================
    // === 附属功能子系统控制函数 ===
    // ==========================================================

    // 控制俯仰角 Servo
    private void setPitchServos(double targetPitch) {
        double clampedLP = Math.max(LP_DOWN, Math.min(LP_UP, targetPitch));
        double proportion = (clampedLP - LP_DOWN) / (LP_UP - LP_DOWN);
        double calculatedRP = RP_DOWN + proportion * (RP_UP - RP_DOWN);
        calculatedRP = Math.max(0.0, Math.min(1.0, calculatedRP));

        LP.setPosition(clampedLP);
        RP.setPosition(calculatedRP);
    }

    public void updateTurret() {
        double currentTicks = turretMotor.getCurrentPosition();
        double currentAngle = currentTicks / TICKS_PER_DEGREE;

        double error = turretTargetAngle - currentAngle;
        double absError = Math.abs(error);

        double dt = pidTimer.seconds();
        if (dt == 0) dt = 0.001;

        double power = 0.0;

        if (absError <= ERROR_TOLERANCE) {
            power = 0;
            integralSum = 0;
            wasInStage1 = false;
        } else if (absError > STAGE_THRESHOLD) {
            wasInStage1 = true;
            integralSum += error * dt;
            double derivative = (error - lastError) / dt;
            power = (kP_far * error) + (kI_far * integralSum) + (kD_far * derivative);
            power = Math.max(-1.0, Math.min(1.0, power));
        } else {
            if (wasInStage1 || Math.signum(error) != Math.signum(lastError)) {
                integralSum = 0;
                wasInStage1 = false;
            }
            if (absError < I_ZONE_near) {
                integralSum += error * dt;
                double maxISum = MAX_INTEGRAL_POWER / (kI_near == 0 ? 1 : kI_near);
                integralSum = Math.max(-maxISum, Math.min(maxISum, integralSum));
            } else {
                integralSum = 0;
            }
            double derivative = (error - lastError) / dt;
            double pidPower = (kP_near * error) + (kI_near * integralSum) + (kD_near * derivative);
            double feedforward = Math.signum(error) * kS_near;

            power = pidPower + feedforward;
            power = Math.max(-1.0, Math.min(1.0, power));
        }

        turretMotor.setPower(power);
        lastError = error;
        pidTimer.reset();
    }

    public void updateFlywheel() {
        double currentVelTPS = motorSH.getVelocity();
        double targetVelTPS = (flywheelTargetRPM * FW_TICKS_PER_REV) / 60.0;

        flywheelCurrentRPM = (currentVelTPS * 60.0) / FW_TICKS_PER_REV;
        double errorRPM = flywheelTargetRPM - flywheelCurrentRPM;

        double dynamicTolerance;
        if (flywheelTargetRPM <= FW_RPM_LOWER_BOUND) {
            dynamicTolerance = FW_MAX_TOLERANCE;
        } else if (flywheelTargetRPM >= FW_RPM_UPPER_BOUND) {
            dynamicTolerance = FW_MIN_TOLERANCE;
        } else {
            double ratio = (flywheelTargetRPM - FW_RPM_LOWER_BOUND) / (FW_RPM_UPPER_BOUND - FW_RPM_LOWER_BOUND);
            dynamicTolerance = FW_MAX_TOLERANCE - ratio * (FW_MAX_TOLERANCE - FW_MIN_TOLERANCE);
        }

        if (flywheelTargetRPM <= 100) {
            isFlywheelReady = false;
        } else {
            if (!isFlywheelReady) {
                if (flywheelCurrentRPM >= flywheelTargetRPM - FW_SPOOL_UP_TOLERANCE) {
                    isFlywheelReady = true;
                }
            } else {
                if (flywheelCurrentRPM < flywheelTargetRPM - dynamicTolerance) {
                    isFlywheelReady = false;
                }
            }
        }

        double dt = flywheelTimer.seconds();
        flywheelTimer.reset();
        if (dt == 0) dt = 1e-9;

        double errorTPS = targetVelTPS - currentVelTPS;

        if (flywheelTargetRPM > 100) {
            flywheelIntegralSum += errorTPS * dt;
        } else {
            flywheelIntegralSum = 0;
        }

        double maxIntegral = 0.25;
        if (FW_kI != 0) {
            if (flywheelIntegralSum > maxIntegral / FW_kI) flywheelIntegralSum = maxIntegral / FW_kI;
            if (flywheelIntegralSum < -maxIntegral / FW_kI) flywheelIntegralSum = -maxIntegral / FW_kI;
        }

        double derivative = (errorTPS - flywheelLastErrorTPS) / dt;
        flywheelLastErrorTPS = errorTPS;

        double power = (FW_kF * targetVelTPS) + (FW_kP * errorTPS) + (FW_kI * flywheelIntegralSum) + (FW_kD * derivative);

        double bangBangThreshold = Math.max(50.0, Math.abs(dynamicTolerance) - (Math.abs(dynamicTolerance) / 7.0));

        if (flywheelTargetRPM > 100) {
            if (errorRPM > bangBangThreshold) {
                power = 1.0;
                flywheelIntegralSum = 0;
            }
        }

        if (flywheelTargetRPM <= 0) {
            power = 0;
            flywheelIntegralSum = 0;
        }

        power = Math.max(0.0, Math.min(1.0, power));

        motorSH.setPower(power);
        motorHS.setPower(power);
    }

    // ==========================================================
    // === 自动射击管理系统 (Non-Blocking) ===
    // ==========================================================

    public void startPrecisionShoot(double durationSeconds) {
        currentShootMode = ShootMode.PRECISION;
        activeShootDuration = durationSeconds;
        shootTimer.reset();
    }

    public void startBlindShoot(double durationSeconds) {
        currentShootMode = ShootMode.BLIND;
        activeShootDuration = durationSeconds;
        shootTimer.reset();
    }

    public boolean isShootingActive() {
        return currentShootMode != ShootMode.NONE;
    }

    public void updateShootingSequence() {
        if (currentShootMode == ShootMode.NONE) {
            return;
        }

        if (shootTimer.seconds() >= activeShootDuration) {
            currentShootMode = ShootMode.NONE;
            bbb.setPosition(0.0);
            intakeMotor.setPower(0.0);
            return;
        }

        if (currentShootMode == ShootMode.BLIND) {
            bbb.setPosition(0.18);
            intakeMotor.setPower(1.0);
        }
        else if (currentShootMode == ShootMode.PRECISION) {
            double errorRPM = Math.abs(flywheelTargetRPM - flywheelCurrentRPM);
            if (errorRPM <= 500) {
                bbb.setPosition(0.18);
                intakeMotor.setPower(1.0);
            } else {
                bbb.setPosition(0.0);
                intakeMotor.setPower(0.0);
            }
        }
    }


    // ==========================================================
    // === 自动流程主状态机 ===
    // ==========================================================
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            // ================= 1. 第一个跑打路径 (diyigepaoda) =================
            case 10:
                follower.setMaxPower(0.7);
                follower.followPath(diyigepaoda, false);
                turretTargetAngle = -50.0;  // ★ 起点：云台设置到 -60 度
                flywheelTargetRPM = 4000.0; // 起点：飞轮4000转
                intakeMotor.setPower(0.0);
                setPathState(11);
                break;
            case 11:
                // 非阻塞等待 0.4 秒后，一边跑一边盲射
                if (pathTimer.getElapsedTimeSeconds() >= 1.6) {
                    startBlindShoot(1); // 持续 0.7 秒
                    setPathState(12);
                }
                break;
            case 12:
                // 等待路径跑完
                if (!follower.isBusy()) {
                    setPathState(20);
                }
                break;

            // ================= 2. 吸第二排 (xidierpai) =================
            case 20:
                follower.setMaxPower(1);
                follower.followPath(xidierpai, false);
                bbb.setPosition(0.0);       // ★ 全程关闭挡板
                intakeMotor.setPower(1.0);  // ★ 开启吸取
                flywheelTargetRPM = 3400; // 降速以节约电能
                setPathState(21);
                break;
            case 21:
                if (!follower.isBusy()) { setPathState(30); }
                break;

            // ================= 3. 发射第二排 (fashedierpai) =================
            case 30:
                follower.followPath(fashedierpai, true);
                turretTargetAngle = -93.0; // ★ 发射前云台调整至 -93 度
                flywheelTargetRPM = 3400.0; // ★ 提升 RPM 到 3400
                intakeMotor.setPower(0.0);  // 跑动时关闭吸取，等待就位后打出
                setPathState(31);
                break;
            case 31:
                if (!follower.isBusy()) {
                    setPathState(315); // 进入非阻塞等待状态
                }
                break;
            case 315:
                // 非阻塞等待 0.5 秒，允许底盘完成微调并且系统继续维持 PID
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                    startPrecisionShoot(1); // ★ 闭环射击 2.0 秒
                    setPathState(32);
                }
                break;
            case 32:
                if (!isShootingActive()) {
                    cycleTimer.resetTimer(); // 从这里开始重置开门15秒计算器
                    setPathState(40);
                }
                break;

            // ================= 4. 开门嘬循环 (kaimenzuo) =================
            case 40:
                follower.followPath(kaimenzuo, true);
                intakeMotor.setPower(1.0);  // ★ 循环期间持续开启 intake
                flywheelTargetRPM = 3400.0; // 回调转速
                setPathState(41);
                break;
            case 41:
                if (!follower.isBusy()) {
                    setPathState(42); // 进入非阻塞等待状态
                }
                break;
            case 42:
                // 在这里非阻塞等待 0.5 秒，允许底盘在结束位置平滑并且维持 PID
                if (pathTimer.getElapsedTimeSeconds() >= 2) {
                    setPathState(50);
                }
                break;

            // ================= 5. 开门嘬返回发射 (fashekaimenzuo) =================
            case 50:
                follower.followPath(fashekaimenzuo, true);
                turretTargetAngle = -93.0; // ★ 开门嘬的发射也是 -93 度
                flywheelTargetRPM = 3400.0; // ★ 转速 3400
                intakeMotor.setPower(1.0);  // ★ 跑回去的时候持续开 intake 保证球进仓
                setPathState(51);
                break;
            case 51:
                if (!follower.isBusy()) {
                    setPathState(515); // 进入非阻塞等待状态
                }
                break;
            case 515:
                // 非阻塞等待 0.5 秒
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                    startPrecisionShoot(1.0); // ★ 闭环射击 2.0 秒
                    setPathState(52);
                }
                break;
            case 52:
                if (!isShootingActive()) {
                    // 判断15秒限制时间
                    if (cycleTimer.getElapsedTimeSeconds() < 15.0) {
                        setPathState(40); // 重新返回开门嘬
                    } else {
                        setPathState(60); // 超时跳出，吸第一排
                    }
                }
                break;

            // ================= 6. 准备吸第一排 (zhunbeixidiyipai) =================
            case 60:
                follower.followPath(zhunbeixidiyipai, false);
                flywheelTargetRPM = 3400.0;
                intakeMotor.setPower(1.0); // ★ 持续开启 intake
                setPathState(61);
                break;
            case 61:
                if (!follower.isBusy() || follower.atParametricEnd()) {
                    setPathState(70);
                }
                break;

            // ================= 7. 吸第一排 (xidiyipai) =================
            case 70:
                follower.followPath(xidiyipai, false);
                intakeMotor.setPower(1.0); // ★ 持续开启 intake
                setPathState(71);
                break;
            case 71:
                if (!follower.isBusy()) { setPathState(80); }
                break;

            // ================= 8. 发射第一排 (fashediyipai) =================
            case 80:
                follower.followPath(fashediyipai, true);
                turretTargetAngle = -93.0; // ★ 第一排发射云台 -93 度
                flywheelTargetRPM = 3400.0; // ★ 转速 3400
                intakeMotor.setPower(0.0);  // 回程停吸取
                setPathState(81);
                break;
            case 81:
                if (!follower.isBusy()) {
                    setPathState(815); // 进入非阻塞等待状态
                }
                break;
            case 815:
                // 非阻塞等待 0.5 秒
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                    startPrecisionShoot(1.0); // ★ 闭环射击 2.0 秒
                    setPathState(82);
                }
                break;
            case 82:
                if (!isShootingActive()) { setPathState(90); }
                break;

            // ================= 9. 准备吸第三排 (zhunbeixidisanpai) =================
            case 90:
                follower.followPath(zhunbeixidisanpai, false);
                flywheelTargetRPM = 3400.0;
                intakeMotor.setPower(1.0); // ★ 持续开启 intake
                setPathState(91);
                break;
            case 91:
                if (!follower.isBusy() || follower.atParametricEnd()) {
                    setPathState(100);
                }
                break;

            // ================= 10. 吸第三排 (xidisanpai) =================
            case 100:
                follower.followPath(xidisanpai, false);
                intakeMotor.setPower(1.0); // ★ 持续开启 intake
                setPathState(101);
                break;
            case 101:
                if (!follower.isBusy()) { setPathState(110); }
                break;

            // ================= 11. 发射第三排 (fashedisanpai) =================
            case 110:
                follower.followPath(fashedisanpai, true);
                turretTargetAngle = -118.0; // ★ 第三排发射云台特殊角度 -122 度
                flywheelTargetRPM = 3400.0; // ★ 转速 3400
                intakeMotor.setPower(0.0);
                setPathState(111);
                break;
            case 111:
                if (!follower.isBusy()) {
                    setPathState(1115); // 进入非阻塞等待状态
                }
                break;
            case 1115:
                // 非阻塞等待 0.5 秒
                if (pathTimer.getElapsedTimeSeconds() >= 0.5) {
                    startPrecisionShoot(1.0); // ★ 闭环射击 2.0 秒
                    setPathState(112);
                }
                break;
            case 112:
                if (!isShootingActive()) { setPathState(120); }
                break;

            // ================= 12. 停靠 (tingkao) =================
            case 120:
                follower.followPath(tingkao, true);
                turretTargetAngle = 0.0; // ★ 停靠时摆正云台到 0
                flywheelTargetRPM = 0.0; // ★ 彻底关闭飞轮
                intakeMotor.setPower(0.0); // ★ 关闭所有吸取机械
                bbb.setPosition(0.0); // ★ 关闭挡板
                setPathState(121);
                break;
            case 121:
                if (!follower.isBusy()) {
                    setPathState(-1); // 彻底结束
                }
                break;
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        cycleTimer = new Timer();
        pidTimer = new ElapsedTime();
        flywheelTimer = new ElapsedTime();
        shootTimer = new ElapsedTime();

        intakeMotor = hardwareMap.get(DcMotorEx.class, "Intake");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        turretMotor = hardwareMap.get(DcMotorEx.class, "Turret");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorSH = hardwareMap.get(DcMotorEx.class, "SH");
        motorHS = hardwareMap.get(DcMotorEx.class, "HS");

        motorSH.setDirection(DcMotorSimple.Direction.FORWARD);
        motorHS.setDirection(DcMotorSimple.Direction.REVERSE);

        motorSH.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorHS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorSH.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorHS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        bbb = hardwareMap.get(Servo.class, "bbb");
        bbb.setPosition(0.0);

        LP = hardwareMap.get(Servo.class, "LP");
        RP = hardwareMap.get(Servo.class, "RP");

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        turretTargetAngle = 0.0;
        flywheelTargetRPM = 0.0;
        currentShootMode = ShootMode.NONE;

        pidTimer.reset();
        flywheelTimer.reset();
    }

    @Override
    public void init_loop() {
        setPitchServos(0.7); // ★ 全程保持俯仰角 0.7
        updateTurret();
        updateFlywheel();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        pidTimer.reset();
        flywheelTimer.reset();
        setPathState(10);
    }

    @Override
    public void loop() {
        // 1. 设置系统硬性姿态要求
        setPitchServos(0.8); // ★ 全程保持俯仰角 0.7

        // 2. 底盘路径与状态更新
        follower.update();
        autonomousPathUpdate();

        // 3. 执行所有 PID 闭环动作
        updateTurret();
        updateFlywheel();

        // 4. 射击控制硬件覆盖（★必须放最后以覆盖基础 intake 指令）
        updateShootingSequence();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Shoot Mode", currentShootMode);
        telemetry.addData("RPM Current/Target", "%.1f / %.1f", flywheelCurrentRPM, flywheelTargetRPM);
        telemetry.update();
    }

    @Override
    public void stop() {
        intakeMotor.setPower(0);
        turretMotor.setPower(0);
        motorSH.setPower(0);
        motorHS.setPower(0);
    }
}