package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.AutoAim.AutoAimSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSubsystem;

@TeleOp(name = "Unlimited TeleOp AirProMaxNeoSuperUltra", group = "Competition")
public class UnlimitedTeleOpAirProMaxNeoUltra extends LinearOpMode {

    private Servo bbb;
    private GoBildaPinpointDriver odo;

    private MecanumDriveSubsystem driveSubsystem;
    private AutoAimSubsystem autoAimSubsystem;
    private FlywheelSubsystem flywheelSubsystem;
    private IntakeSubsystem intakeSubsystem;

    private final double TARGET_X_WORLD = 135.0;
    private final double TARGET_Y_WORLD = 136.0;

    private boolean isShootingMode = false;
    private boolean lastCircleState = false;

    // 修改：删除了 lastCrossState 和 isPreSpoolingMode，因为现在手动模式默认预热
    private boolean isManualMode = false;
    private boolean lastLeftBumperState = false;
    private boolean lastSquareState = false;

    private double manualTargetDistance = 25.0;
    // 修改：删除了 manualIdleOverride，现在策略是实时根据距离结算

    private double headingOffset = 0.0;

    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        bbb = hardwareMap.get(Servo.class, "bbb");
        bbb.setPosition(0);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(101.16, -160, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
        sleep(300);
        odo.setPosition(new Pose2D(DistanceUnit.INCH, 72.0, 72.0, AngleUnit.DEGREES, 0.0));
        odo.update();

        driveSubsystem = new MecanumDriveSubsystem(hardwareMap);
        autoAimSubsystem = new AutoAimSubsystem(hardwareMap);
        flywheelSubsystem = new FlywheelSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);

        telemetry.addLine("Ready to Start - Drive, Odometry, Turret, Flywheel & Intake Active!");
        telemetry.update();

        waitForStart();

        flywheelSubsystem.start();
        intakeSubsystem.start();

        while (opModeIsActive()) {
            odo.update();
            Pose2D pos = odo.getPosition();

            double rx_odo = pos.getX(DistanceUnit.INCH);
            double ry_odo = pos.getY(DistanceUnit.INCH);
            double rawHeadingDeg = pos.getHeading(AngleUnit.DEGREES);
            double currentHeadingDeg = rawHeadingDeg - headingOffset;

            double robotVx = odo.getVelX(DistanceUnit.INCH);
            double robotVy = odo.getVelY(DistanceUnit.INCH);
            double headingRadForVel = Math.toRadians(currentHeadingDeg);

            double globalVx = robotVx * Math.cos(headingRadForVel) - robotVy * Math.sin(headingRadForVel);
            double globalVy = robotVx * Math.sin(headingRadForVel) + robotVy * Math.cos(headingRadForVel);

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx_drive = gamepad1.right_stick_x;

            if (gamepad1.right_stick_button) {
                headingOffset = rawHeadingDeg;
            }

            driveSubsystem.driveFieldCentric(x, y, rx_drive, currentHeadingDeg);

            // Square (X) 退出射击模式
            boolean currentSquareState = gamepad1.x;
            if (currentSquareState && !lastSquareState) {
                isShootingMode = false;
            }
            lastSquareState = currentSquareState;

            // Circle (B) 开启/切换射击模式
            boolean currentCircleState = gamepad1.b;
            if (currentCircleState && !lastCircleState) {
                isShootingMode = !isShootingMode;
            }
            lastCircleState = currentCircleState;

            // Left Bumper 切换手动/自动自瞄档
            boolean currentLeftBumperState = gamepad1.left_bumper;
            if (currentLeftBumperState && !lastLeftBumperState) {
                isManualMode = !isManualMode;
            }
            lastLeftBumperState = currentLeftBumperState;

            // 手动模式下的预设距离切换
            if (isManualMode) {
                if (gamepad1.dpad_left) { manualTargetDistance = 25.0; }
                else if (gamepad1.dpad_up) { manualTargetDistance = 54.23; }
                else if (gamepad1.dpad_right) { manualTargetDistance = 88.0; }
                else if (gamepad1.dpad_down) { manualTargetDistance = 150.0; }
            }

            boolean isEmergencyBrake = gamepad1.right_bumper;
            double robotOmega = odo.getHeadingVelocity(AngleUnit.DEGREES.getUnnormalized());

            // 获取自瞄结算
            AutoAimSubsystem.TurretCommand aimCommand = autoAimSubsystem.update(
                    rx_odo, ry_odo, globalVx, globalVy, currentHeadingDeg, robotOmega,
                    TARGET_X_WORLD, TARGET_Y_WORLD,
                    isManualMode, manualTargetDistance
            );

            double targetVelocityRPM = FlywheelSubsystem.IDLE_VELOCITY_MIN;
            String flywheelActionState = "怠速 (Idle)";

            if (isEmergencyBrake) {
                targetVelocityRPM = 0;
                flywheelActionState = "紧急刹车 (E-Brake/电制动)";
                bbb.setPosition(0);
            } else if (isShootingMode) {
                // 射击模式：完全释放结算转速，上限 4900
                bbb.setPosition(0.18);
                if (aimCommand.hasTarget) {
                    targetVelocityRPM = Math.min(FlywheelSubsystem.SHOOT_MAX, aimCommand.targetRpm);
                    flywheelActionState = "开火中 (Shooting - 全速解锁)";
                }
            } else if (isManualMode) {
                // 手动档默认实时预热：限制在 [2900, 4000]
                bbb.setPosition(0);
                if (aimCommand.hasTarget) {
                    // 实时结算但限制在 4000 以下
                    double calculatedRpm = aimCommand.targetRpm;
                    targetVelocityRPM = Math.max(FlywheelSubsystem.IDLE_VELOCITY_MIN, Math.min(FlywheelSubsystem.PRE_SPOOL_MAX, calculatedRpm));
                    flywheelActionState = "自适应预热 (2900-4000 动态限速)";
                } else {
                    targetVelocityRPM = FlywheelSubsystem.IDLE_VELOCITY_MIN;
                    flywheelActionState = "手动底噪怠速 (2900)";
                }
            } else {
                // 非手动且非射击：进入标准低功耗怠速
                bbb.setPosition(0);
                targetVelocityRPM = FlywheelSubsystem.IDLE_VELOCITY_MIN;
                flywheelActionState = "自动档空闲 (2900)";
            }

            // 更新飞轮：在手动模式或射击模式下，都视为 isActiveSpooling = true (用于判断就绪和应用 Bang-Bang)
            boolean isActiveSpooling = (isShootingMode || isManualMode) && !isEmergencyBrake;
            flywheelSubsystem.update(targetVelocityRPM, isEmergencyBrake, isActiveSpooling);

            boolean rpmOK = flywheelSubsystem.isReady();
            boolean effectiveAimLocked = isManualMode ? true : aimCommand.isAimLocked;

            intakeSubsystem.update(isShootingMode, aimCommand.hasTarget, aimCommand.isUnwinding, effectiveAimLocked, rpmOK, aimCommand.targetDist);

            // Telemetry 更新
            telemetry.addData("操作模式", isManualMode ? "🛠️ [手动控制档] (自适应预热激活)" : "🤖 [自动自瞄档]");
            telemetry.addData("当前动作", flywheelActionState);

            telemetry.addLine("--- 实时控制状态 ---");
            if (aimCommand.hasTarget) {
                telemetry.addData("理论结算转速", "%.1f RPM", aimCommand.targetRpm);
                telemetry.addData("目标距离", "%.1f in", aimCommand.targetDist);
            }
            telemetry.addData("飞轮目标 RPM", "%.1f", targetVelocityRPM);
            telemetry.addData("飞轮当前 RPM", "%.1f", flywheelSubsystem.getCurrentRPM());
            telemetry.addData("状态", rpmOK ? "✅ READY" : "⏳ 调整中 (支持电制动降速)");
            telemetry.addData("加速策略", flywheelSubsystem.getActiveBoostState());

            telemetry.update();
        }

        autoAimSubsystem.stop();
        driveSubsystem.stop();
    }
}