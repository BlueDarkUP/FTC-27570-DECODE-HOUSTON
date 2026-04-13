package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private final double TARGET_X_WORLD = 136.0;
    private final double TARGET_Y_WORLD = 135.0;

    private final double IDLE_VELOCITY = 3000.0;

    // 引入线性映射的动态容差参数：转速越高，要求越严，容差越小
    private final double RPM_LOWER_BOUND = 3000.0;
    private final double RPM_UPPER_BOUND = 5050.0;
    private final double MAX_TOLERANCE = 1000;
    private final double MIN_TOLERANCE = 1000;

    // 新增：首次蓄力完成的极小容差（必须达到目标转速 - 100 才能首次开火）
    private final double SPOOL_UP_TOLERANCE = 100.0;

    private final double kP = 0.011;
    private final double kI = 0.0004;
    private final double kD = 0.00000023;
    private final double kF = 0.00033;
    private final double TICKS_PER_REV = 28.0;

    private final double STALL_CURRENT_AMPS = 2.45;
    private final double STALL_COOLDOWN_SEC = 0.5;
    private final double STALL_TIME_THRESHOLD_SEC = 0.3;

    private final double LP_UP = 1;
    private final double LP_DOWN = 0.4;
    private final double RP_UP = 0;
    private final double RP_DOWN = 0.5;

    private boolean isShootingMode = false;
    private boolean lastCircleState = false;

    // 新增：记录飞轮是否已经蓄力完成的状态标志
    private boolean isFlywheelReady = false;

    private double intakeBrakeReleaseTime = 0.0;

    private double stallStartTime = 0.0;
    private boolean isStalling = false;

    private double unwindReverseEndTime = 0.0;
    private boolean wasUnwinding = false;

    private ElapsedTime timer = new ElapsedTime();
    private double lastErrorTPS = 0;
    private double integralSum = 0;

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

        timer.reset();

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            lf.setPower((y + x + rx) / denominator);
            lb.setPower((y - x + rx) / denominator);
            rf.setPower((y - x - rx) / denominator);
            rb.setPower((y + x - rx) / denominator);

            AutoAimSubsystem.TurretCommand aimCommand = autoAim.update(TARGET_X_WORLD, TARGET_Y_WORLD);

            if (aimCommand.hasTarget) {
                setPitchServos(aimCommand.targetPitch);
            }

            boolean currentCircleState = gamepad1.b;
            if (currentCircleState && !lastCircleState) {
                isShootingMode = !isShootingMode;
            }
            lastCircleState = currentCircleState;

            double targetVelocityRPM = IDLE_VELOCITY;
            if (isShootingMode) {
                bbb.setPosition(0.18);
                if (aimCommand.hasTarget) {
                    targetVelocityRPM = aimCommand.targetRpm;
                }
            } else {
                bbb.setPosition(0);
            }

            double dynamicTolerance;
            if (targetVelocityRPM <= RPM_LOWER_BOUND) {
                dynamicTolerance = MAX_TOLERANCE;
            } else if (targetVelocityRPM >= RPM_UPPER_BOUND) {
                dynamicTolerance = MIN_TOLERANCE;
            } else {
                double ratio = (targetVelocityRPM - RPM_LOWER_BOUND) / (RPM_UPPER_BOUND - RPM_LOWER_BOUND);
                dynamicTolerance = MAX_TOLERANCE - ratio * (MAX_TOLERANCE - MIN_TOLERANCE);
            }

            double currentVelTPS = motorSH.getVelocity();
            double targetVelTPS = (targetVelocityRPM * TICKS_PER_REV) / 60.0;
            double currentRPM = (currentVelTPS * 60.0) / TICKS_PER_REV;
            double errorRPM = targetVelocityRPM - currentRPM;

            // ================= 新增：飞轮迟滞与状态机逻辑 =================
            if (!isShootingMode) {
                // 不在射击模式时，重置飞轮就绪状态
                isFlywheelReady = false;
            } else {
                if (!isFlywheelReady) {
                    // 【蓄力阶段】必须非常接近目标转速才能首次射击（允许少量误差如100RPM防卡死）
                    if (currentRPM >= targetVelocityRPM - SPOOL_UP_TOLERANCE) {
                        isFlywheelReady = true;
                    }
                } else {
                    // 【连发阶段】一旦开始射击，允许转速下掉，只要掉速不超过动态容差即可
                    if (currentRPM < targetVelocityRPM - dynamicTolerance) {
                        isFlywheelReady = false; // 掉速严重，强制停止推弹重新蓄力
                    }
                }
            }
            // 使用状态机结果替代原先的直接判断
            boolean rpmOK = isFlywheelReady;
            // ==============================================================

            double dt = timer.seconds();
            timer.reset();
            if (dt == 0) dt = 1e-9;

            double errorTPS = targetVelTPS - currentVelTPS;

            if (targetVelocityRPM > 100) {
                integralSum += errorTPS * dt;
            } else {
                integralSum = 0;
            }

            double maxIntegral = 0.25;
            if (kI != 0) {
                if (integralSum > maxIntegral / kI) integralSum = maxIntegral / kI;
                if (integralSum < -maxIntegral / kI) integralSum = -maxIntegral / kI;
            }

            double derivative = (errorTPS - lastErrorTPS) / dt;
            lastErrorTPS = errorTPS;

            double power = (kF * targetVelTPS) + (kP * errorTPS) + (kI * integralSum) + (kD * derivative);

            double bangBangThreshold = Math.abs(dynamicTolerance) - (Math.abs(dynamicTolerance) / 7.0);
            String activeBoost = "None";

            if (isShootingMode && targetVelocityRPM > 100) {
                if (errorRPM > bangBangThreshold) {
                    power = 1.0;
                    integralSum = 0;
                    activeBoost = "Bang-Bang (极速补电)";
                }
            }

            if (targetVelocityRPM <= 0) {
                power = 0;
                integralSum = 0;
            }

            power = Math.max(0.0, Math.min(1.0, power));

            motorSH.setPower(power);
            motorHS.setPower(power);

            double intakeCurrent = motorIntake.getCurrent(CurrentUnit.AMPS);

            if (isShootingMode) {
                isStalling = false;

                if (aimCommand.hasTarget && aimCommand.isUnwinding) {
                    if (!wasUnwinding) {
                        unwindReverseEndTime = getRuntime() + 0.35;
                    }

                    if (getRuntime() < unwindReverseEndTime) {
                        motorIntake.setPower(-1.0);
                        telemetry.addData("🔴 发射系统", "⚠️ 云台复位中: Intake 反转退弹!");
                    } else {
                        motorIntake.setPower(0.0);
                        telemetry.addData("🔴 发射系统", "⚠️ 云台复位中: Intake 刹车等待");
                    }
                    intakeBrakeReleaseTime = 0.0;

                } else {
                    if (aimCommand.hasTarget && rpmOK && aimCommand.isAimLocked) {
                        motorIntake.setPower(1.0);
                        telemetry.addData("🔴 发射系统", "⚡ 动态射击中 (FIRE ON THE MOVE)!");
                    } else {
                        motorIntake.setPower(0.0);
                        if (!aimCommand.hasTarget) {
                            telemetry.addData("🔴 发射系统", "等待目标...");
                        } else if (!rpmOK) {
                            telemetry.addData("🔴 发射系统", "飞轮蓄力调速中 (等待达到满转)...");
                        } else if (!aimCommand.isAimLocked) {
                            telemetry.addData("🔴 发射系统", "云台动态追瞄中 (等待角度容差锁定)...");
                        }
                    }
                }

            } else {
                if (intakeCurrent >= STALL_CURRENT_AMPS) {
                    if (!isStalling) {
                        isStalling = true;
                        stallStartTime = getRuntime();
                    } else if (getRuntime() - stallStartTime >= STALL_TIME_THRESHOLD_SEC) {
                        intakeBrakeReleaseTime = getRuntime() + STALL_COOLDOWN_SEC;
                    }
                } else {
                    isStalling = false;
                }

                if (getRuntime() < intakeBrakeReleaseTime) {
                    motorIntake.setPower(0.0);
                    telemetry.addData("🟢 发射系统", "⚠️ INTAKE堵转保护触发！(刹车冷却中)");
                    isStalling = false;
                } else {
                    motorIntake.setPower(0.8);
                    telemetry.addData("🟢 发射系统", "怠速中 (Intake 常转收件)");
                }
            }

            wasUnwinding = aimCommand.isUnwinding;

            // [八] 遥测输出
            telemetry.addData("当前模式", isShootingMode ? "[ 发射模式 ]" : "[ 怠速模式 ]");
            telemetry.addData("瞬态接管状态", activeBoost);
            if (aimCommand.hasTarget) {
                telemetry.addData("Pitch 舵机", "LP: %.3f | RP: %.3f", LP.getPosition(), RP.getPosition());
            }
            telemetry.addData("飞轮当前 RPM", currentRPM);
            telemetry.addData("飞轮目标 RPM", targetVelocityRPM);
            telemetry.addData("RPM 误差", errorRPM);

            // 优化了遥测信息，方便驾驶员知道当前飞轮是处于等待状态还是射击状态
            telemetry.addData("飞轮状态", isFlywheelReady ? "✅ 已满转 (允许掉速射击)" : "⏳ 蓄力中 (等待达到满转)...");
            telemetry.addData("允许最大掉速 (容差)", "-%.1f RPM", dynamicTolerance);

            telemetry.addData("极速补电触发线", "> %.1f", bangBangThreshold);
            telemetry.addData("当前飞轮动力分配", "%.2f", power);
            telemetry.addData("Intake 电流 (A)", "%.2f", intakeCurrent);

            if (!isShootingMode && isStalling && getRuntime() >= intakeBrakeReleaseTime) {
                telemetry.addData("⚠️ 堵转警告", "高电流持续中: %.2f / %.2f 秒",
                        (getRuntime() - stallStartTime), STALL_TIME_THRESHOLD_SEC);
            }

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