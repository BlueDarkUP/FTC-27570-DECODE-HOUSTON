package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
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

import org.firstinspires.ftc.teamcode.AutoAim.ManualAimSubsystem;

@TeleOp(name = "Unlimited TeleOp AirProMaxNeoSuperUltra", group = "Competition")
public class UnlimitedTeleOpAirProMaxNeoUltra extends LinearOpMode {

    // 硬件声明
    private DcMotor lf = null, rf = null, lb = null, rb = null;
    private DcMotorEx motorSH, motorHS, motorIntake;
    private Servo bbb;
    private GoBildaPinpointDriver odo; // 替代原来的 IMU

    // 引入自瞄子系统
    private ManualAimSubsystem autoAimSubsystem;

    // 坐标与飞轮常量
    private final double TARGET_X_WORLD = 135.0;
    private final double TARGET_Y_WORLD = 136.0;
    private final double IDLE_VELOCITY = 3000.0;
    private final double TRIGGER_DEADZONE = 0.3;

    private final double RPM_LOWER_BOUND = 3000.0;
    private final double RPM_UPPER_BOUND = 5050.0;
    private final double MAX_TOLERANCE = 1000;
    private final double MIN_TOLERANCE = 1000;
    private final double SPOOL_UP_TOLERANCE = 100.0;

    private final double kP = 0.011, kI = 0.0004, kD = 0.00000023, kF = 0.00033;
    private final double TICKS_PER_REV = 28.0;

    private final double STALL_CURRENT_AMPS = 2.45;
    private final double STALL_COOLDOWN_SEC = 0.5;
    private final double STALL_TIME_THRESHOLD_SEC = 0.3;

    // 状态机变量
    private boolean isShootingMode = false;
    private boolean lastCircleState = false;
    private boolean isFlywheelReady = false;

    private boolean isManualMode = false;
    private boolean lastLeftBumperState = false;
    private boolean lastSquareState = false;
    private double manualTargetDistance = 25.0;
    private boolean manualIdleOverride = false;

    private double intakeBrakeReleaseTime = 0.0;
    private double stallStartTime = 0.0;
    private boolean isStalling = false;
    private double unwindReverseEndTime = 0.0;
    private boolean wasUnwinding = false;

    private ElapsedTime timer = new ElapsedTime();
    private double lastErrorTPS = 0;
    private double integralSum = 0;

    // 按键重置用变量
    private double headingOffset = 0.0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // 1. 初始化底盘
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");

        lf.setDirection(DcMotor.Direction.REVERSE); lb.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD); rb.setDirection(DcMotor.Direction.FORWARD);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 2. 初始化发射系统
        motorSH = hardwareMap.get(DcMotorEx.class, "SH"); motorHS = hardwareMap.get(DcMotorEx.class, "HS");
        motorIntake = hardwareMap.get(DcMotorEx.class, "Intake");
        bbb = hardwareMap.get(Servo.class, "bbb");

        motorSH.setDirection(DcMotorSimple.Direction.FORWARD); motorHS.setDirection(DcMotorSimple.Direction.REVERSE);
        motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);

        motorSH.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); motorHS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorSH.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT); motorHS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        bbb.setPosition(0);

        // 3. 初始化 Pinpoint (代替原有的 IMU 和 Odo)
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(101.16, -160, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
        sleep(300);
        odo.setPosition(new Pose2D(DistanceUnit.INCH, 72.0, 72.0, AngleUnit.DEGREES, 0.0));
        odo.update();

        // 4. 初始化云台自瞄子系统 (封装版)
        autoAimSubsystem = new ManualAimSubsystem(hardwareMap);

        telemetry.addLine("Ready to Start - Odometry & Turret are Active!");
        telemetry.update();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            // ============== 1. 获取定位数据与物理运算 ==============
            odo.update();
            Pose2D pos = odo.getPosition();

            double rx_odo = pos.getX(DistanceUnit.INCH);
            double ry_odo = pos.getY(DistanceUnit.INCH);
            // Pinpoint自身返回的角度
            double rawHeadingDeg = pos.getHeading(AngleUnit.DEGREES);

            // 组合按键重置Yaw功能的真实Heading
            double currentHeadingDeg = rawHeadingDeg - headingOffset;

            // 速度转换
            double robotVx = odo.getVelX(DistanceUnit.INCH);
            double robotVy = odo.getVelY(DistanceUnit.INCH);
            double headingRadForVel = Math.toRadians(currentHeadingDeg);

            double globalVx = robotVx * Math.cos(headingRadForVel) - robotVy * Math.sin(headingRadForVel);
            double globalVy = robotVx * Math.sin(headingRadForVel) + robotVy * Math.cos(headingRadForVel);

            // ============== 2. 底盘控制 (使用精确的Pinpoint作为无头模式) ==============
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx_drive = gamepad1.right_stick_x;

            if (gamepad1.right_stick_button) {
                headingOffset = rawHeadingDeg; // 虚拟重置航向为0
            }

            // 无头模式场控转换
            double currentHeadingRad = Math.toRadians(currentHeadingDeg);
            double rotX = x * Math.cos(-currentHeadingRad) - y * Math.sin(-currentHeadingRad);
            double rotY = x * Math.sin(-currentHeadingRad) + y * Math.cos(-currentHeadingRad);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx_drive), 1.0);
            lf.setPower((rotY + rotX + rx_drive) / denominator);
            lb.setPower((rotY - rotX + rx_drive) / denominator);
            rf.setPower((rotY - rotX - rx_drive) / denominator);
            rb.setPower((rotY + rotX - rx_drive) / denominator);

            // ============== 3. 按键逻辑与状态机 ==============
            boolean currentSquareState = gamepad1.x;
            if (currentSquareState && !lastSquareState) {
                isShootingMode = false;
                if (isManualMode) manualIdleOverride = true;
            }
            lastSquareState = currentSquareState;

            boolean currentCircleState = gamepad1.b;
            if (currentCircleState && !lastCircleState) {
                isShootingMode = !isShootingMode;
            }
            lastCircleState = currentCircleState;

            boolean currentLeftBumperState = gamepad1.left_bumper;
            if (currentLeftBumperState && !lastLeftBumperState) {
                isManualMode = !isManualMode;
                if (isManualMode) manualIdleOverride = false;
            }
            lastLeftBumperState = currentLeftBumperState;

            if (isManualMode) {
                if (gamepad1.dpad_left) { manualTargetDistance = 25.0; manualIdleOverride = false; }
                else if (gamepad1.dpad_up) { manualTargetDistance = 54.23; manualIdleOverride = false; }
                else if (gamepad1.dpad_right) { manualTargetDistance = 88.0; manualIdleOverride = false; }
                else if (gamepad1.dpad_down) { manualTargetDistance = 150.0; manualIdleOverride = false; }
            }

            boolean isEmergencyBrake = gamepad1.right_bumper;
            boolean isPreSpooling = (!isManualMode && gamepad1.left_trigger > TRIGGER_DEADZONE);

            // ============== 4. 驱动自瞄子系统 ==============
            ManualAimSubsystem.TurretCommand aimCommand = autoAimSubsystem.update(
                    rx_odo, ry_odo, globalVx, globalVy, currentHeadingDeg,
                    TARGET_X_WORLD, TARGET_Y_WORLD,
                    isManualMode, manualTargetDistance
            );

            // ============== 5. 飞轮状态控制 ==============
            double targetVelocityRPM = IDLE_VELOCITY;
            String flywheelActionState = "怠速 (Idle)";

            if (isEmergencyBrake) {
                targetVelocityRPM = 0;
                flywheelActionState = "紧急刹车 (E-Brake)";
                bbb.setPosition(0);
            } else if (isShootingMode) {
                bbb.setPosition(0.18);
                if (aimCommand.hasTarget) {
                    targetVelocityRPM = aimCommand.targetRpm;
                    flywheelActionState = "开火中 (Shooting)";
                }
            } else {
                bbb.setPosition(0);
                if (isManualMode && !manualIdleOverride && aimCommand.hasTarget) {
                    targetVelocityRPM = aimCommand.targetRpm;
                    flywheelActionState = "手动档持续预热 (Manual Spooling)";
                } else if (isPreSpooling && aimCommand.hasTarget) {
                    targetVelocityRPM = aimCommand.targetRpm;
                    flywheelActionState = "战斗姿态预蓄力 (Pre-spooling)";
                } else {
                    targetVelocityRPM = IDLE_VELOCITY;
                    flywheelActionState = "怠速 (Idle)";
                }
            }

            // 飞轮 PID 控制与容差计算
            double dynamicTolerance;
            if (targetVelocityRPM <= RPM_LOWER_BOUND) dynamicTolerance = MAX_TOLERANCE;
            else if (targetVelocityRPM >= RPM_UPPER_BOUND) dynamicTolerance = MIN_TOLERANCE;
            else {
                double ratio = (targetVelocityRPM - RPM_LOWER_BOUND) / (RPM_UPPER_BOUND - RPM_LOWER_BOUND);
                dynamicTolerance = MAX_TOLERANCE - ratio * (MAX_TOLERANCE - MIN_TOLERANCE);
            }

            double currentVelTPS = motorSH.getVelocity();
            double targetVelTPS = (targetVelocityRPM * TICKS_PER_REV) / 60.0;
            double currentRPM = (currentVelTPS * 60.0) / TICKS_PER_REV;
            double errorRPM = targetVelocityRPM - currentRPM;

            if (!isShootingMode && !isPreSpooling && (!isManualMode || manualIdleOverride)) {
                isFlywheelReady = false;
            } else {
                if (!isFlywheelReady) {
                    if (currentRPM >= targetVelocityRPM - SPOOL_UP_TOLERANCE) isFlywheelReady = true;
                } else {
                    if (currentRPM < targetVelocityRPM - dynamicTolerance) isFlywheelReady = false;
                }
            }
            boolean rpmOK = isFlywheelReady;

            // PIDF 运算
            double dt = timer.seconds();
            timer.reset();
            if (dt == 0) dt = 1e-9;
            double errorTPS = targetVelTPS - currentVelTPS;

            if (targetVelocityRPM > 100) integralSum += errorTPS * dt;
            else integralSum = 0;

            double maxIntegral = 0.25;
            if (kI != 0) {
                if (integralSum > maxIntegral / kI) integralSum = maxIntegral / kI;
                if (integralSum < -maxIntegral / kI) integralSum = -maxIntegral / kI;
            }

            double derivative = (errorTPS - lastErrorTPS) / dt;
            lastErrorTPS = errorTPS;
            double power = (kF * targetVelTPS) + (kP * errorTPS) + (kI * integralSum) + (kD * derivative);

            double bangBangThreshold = Math.max(50.0, Math.abs(dynamicTolerance) - (Math.abs(dynamicTolerance) / 7.0));
            String activeBoost = "None";

            if ((isShootingMode || isPreSpooling || (isManualMode && !manualIdleOverride)) && targetVelocityRPM > 100) {
                if (errorRPM > bangBangThreshold) {
                    power = 1.0; integralSum = 0; activeBoost = "Bang-Bang (极速补电)";
                }
            }

            if (targetVelocityRPM <= 0 || isEmergencyBrake) {
                power = 0; integralSum = 0;
            }

            power = Math.max(0.0, Math.min(1.0, power));
            motorSH.setPower(power); motorHS.setPower(power);

            // ============== 6. 进件器 Intake 控制与安全机制 ==============
            double intakeCurrent = motorIntake.getCurrent(CurrentUnit.AMPS);

            if (isShootingMode) {
                isStalling = false;
                if (aimCommand.hasTarget && aimCommand.isUnwinding) {
                    if (!wasUnwinding) unwindReverseEndTime = getRuntime() + 0.35;
                    if (getRuntime() < unwindReverseEndTime) {
                        motorIntake.setPower(-0.2);
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
                        if (!aimCommand.hasTarget) telemetry.addData("🔴 发射系统", "等待目标 (或处于刹车保护)...");
                        else if (!rpmOK) telemetry.addData("🔴 发射系统", "飞轮蓄力调速中 (等待达到满转)...");
                        else if (!aimCommand.isAimLocked) telemetry.addData("🔴 发射系统", "云台动态追瞄中 (等待角度容差锁定)...");
                    }
                }
            } else {
                if (intakeCurrent >= STALL_CURRENT_AMPS) {
                    if (!isStalling) { isStalling = true; stallStartTime = getRuntime(); }
                    else if (getRuntime() - stallStartTime >= STALL_TIME_THRESHOLD_SEC) {
                        intakeBrakeReleaseTime = getRuntime() + STALL_COOLDOWN_SEC;
                    }
                } else isStalling = false;

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

            // ============== 7. Telemetry 数据更新 ==============
            telemetry.addData("操作模式", isManualMode ? "🛠️ [手动控制档] (屏蔽动态预测)" : "🤖 [自动自瞄档]");
            telemetry.addData("当前动作", isShootingMode ? "[ 发射模式 ]" : "[ 怠速/收集模式 ]");
            telemetry.addData("飞轮动作策略", flywheelActionState);
            telemetry.addData("瞬态接管状态", activeBoost);

            telemetry.addData("Odo X / Y", "%.1f / %.1f", rx_odo, ry_odo);
            telemetry.addData("Heading", "%.1f", currentHeadingDeg);

            if (aimCommand.hasTarget) {
                telemetry.addData("云台锁定", aimCommand.isAimLocked ? "✅ LOCKED" : "⏳ 追踪中");
                telemetry.addData("计算 Pitch", "%.3f", aimCommand.targetPitch);
            }

            telemetry.addData("飞轮目标 RPM", targetVelocityRPM);
            telemetry.addData("飞轮当前 RPM", currentRPM);
            telemetry.addData("飞轮状态", isFlywheelReady ? "✅ 已满转" : "⏳ 蓄力/怠速中...");

            telemetry.update();
        }

        // OPMode 结束后停止子系统电机
        autoAimSubsystem.stop();
    }
}