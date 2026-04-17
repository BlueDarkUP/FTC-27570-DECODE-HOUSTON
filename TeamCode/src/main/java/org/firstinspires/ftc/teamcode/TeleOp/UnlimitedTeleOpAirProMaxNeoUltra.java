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

    private boolean isPreSpoolingMode = false;
    private boolean lastCrossState = false;

    private boolean isManualMode = false;
    private boolean lastLeftBumperState = false;
    private boolean lastSquareState = false;
    private double manualTargetDistance = 25.0;
    private boolean manualIdleOverride = false;

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

        telemetry.addLine("Ready to Start - Drive, Odometry, Turret, Flywheel & Intake are Active!");
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

            boolean currentSquareState = gamepad1.x;
            if (currentSquareState && !lastSquareState) {
                isShootingMode = false;
                isPreSpoolingMode = false;
                if (isManualMode) manualIdleOverride = true;
            }
            lastSquareState = currentSquareState;

            boolean currentCircleState = gamepad1.b;
            if (currentCircleState && !lastCircleState) {
                isShootingMode = !isShootingMode;
            }
            lastCircleState = currentCircleState;

            boolean currentCrossState = gamepad1.a;
            if (currentCrossState && !lastCrossState) {
                isPreSpoolingMode = !isPreSpoolingMode;
            }
            lastCrossState = currentCrossState;

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
            boolean isPreSpoolingActive = (!isManualMode && isPreSpoolingMode);

            double robotOmega = odo.getHeadingVelocity(AngleUnit.DEGREES.getUnnormalized());
            AutoAimSubsystem.TurretCommand aimCommand = autoAimSubsystem.update(
                    rx_odo, ry_odo, globalVx, globalVy, currentHeadingDeg,robotOmega,
                    TARGET_X_WORLD, TARGET_Y_WORLD,
                    isManualMode, manualTargetDistance
            );

            double targetVelocityRPM = FlywheelSubsystem.IDLE_VELOCITY;
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
                } else if (isPreSpoolingActive && aimCommand.hasTarget) {
                    targetVelocityRPM = aimCommand.targetRpm;
                    flywheelActionState = "战斗姿态预蓄力 (Pre-spooling) [Cross键已锁定]";
                } else {
                    targetVelocityRPM = FlywheelSubsystem.IDLE_VELOCITY;
                    flywheelActionState = "怠速 (Idle)";
                }
            }

            boolean isActiveSpooling = (!isShootingMode && !isPreSpoolingActive && (!isManualMode || manualIdleOverride)) ? false : true;

            flywheelSubsystem.update(targetVelocityRPM, isEmergencyBrake, isActiveSpooling);
            boolean rpmOK = flywheelSubsystem.isReady();

            boolean effectiveAimLocked = isManualMode ? true : aimCommand.isAimLocked;

            intakeSubsystem.update(isShootingMode, aimCommand.hasTarget, aimCommand.isUnwinding, effectiveAimLocked, rpmOK);

            telemetry.addData("操作模式", isManualMode ? "🛠️ [手动控制档] (屏蔽动态预测)" : "🤖 [自动自瞄档]");
            telemetry.addData("当前动作", isShootingMode ? "[ 发射模式 ]" : "[ 怠速/收集模式 ]");

            telemetry.addData("Odo X / Y", "%.1f / %.1f", rx_odo, ry_odo);
            telemetry.addData("Heading", "%.1f", currentHeadingDeg);

            telemetry.addLine("--- 发射系统状态 ---");
            if (aimCommand.hasTarget) {
                telemetry.addData("云台锁定", effectiveAimLocked ? "✅ LOCKED" : "⏳ 追踪中");
                telemetry.addData("计算 Pitch", "%.3f", aimCommand.targetPitch);
            }
            telemetry.addData("推弹机构", isShootingMode ? "发射推弹态 (0.18)" : "归位态 (0.0)");
            telemetry.addData("Intake 状态", intakeSubsystem.getStatusMessage());

            telemetry.addLine("--- 飞轮系统状态 ---");
            telemetry.addData("飞轮策略", flywheelActionState);
            telemetry.addData("瞬态接管", flywheelSubsystem.getActiveBoostState());
            telemetry.addData("飞轮目标 RPM", targetVelocityRPM);
            telemetry.addData("飞轮当前 RPM", flywheelSubsystem.getCurrentRPM());
            telemetry.addData("飞轮转速是否达标", rpmOK ? "✅ 已满转" : "⏳ 蓄力/怠速中...");

            telemetry.update();
        }

        autoAimSubsystem.stop();
        driveSubsystem.stop();
    }
}