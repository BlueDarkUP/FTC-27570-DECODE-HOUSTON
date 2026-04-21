package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.AutoAim.AutoAimSubsystem;
import org.firstinspires.ftc.teamcode.DeadEye.LimelightPinpointLocalizer;
import org.firstinspires.ftc.teamcode.Subsystems.FlywheelSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.MecanumDriveSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ClimbSubsystem;
import org.firstinspires.ftc.teamcode.GlobalConstants;

public abstract class BaseTeleOp extends LinearOpMode {

    private Servo bbb;
    private GoBildaPinpointDriver odo;

    private LimelightPinpointLocalizer visionLocalizer;

    private MecanumDriveSubsystem driveSubsystem;
    private AutoAimSubsystem autoAimSubsystem;
    private FlywheelSubsystem flywheelSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private ClimbSubsystem climbSubsystem;
    protected double TARGET_X_WORLD;
    protected double TARGET_Y_WORLD;
    protected double headingOffset = 0.0;

    protected abstract double getTargetX();
    protected abstract double getTargetY();
    protected abstract double getHeadingOffset();

    private boolean isShootingMode = false;
    private boolean lastCircleState = false;
    private boolean isManualMode = false;
    private boolean lastLeftBumperState = false;
    private boolean lastSquareState = false;
    private boolean lastRightStickButton = false;
    private boolean lastBackState = false;

    private double manualTargetDistance = 25.0;

    private boolean lastConditionsMet = false;
    private long lastRumbleTime = 0;
    private boolean visionCalibrationSuccess = false;

    private ElapsedTime bbbTimer = new ElapsedTime();

    @Override
    public void runOpMode() {
        TARGET_X_WORLD = getTargetX();
        TARGET_Y_WORLD = getTargetY();
        headingOffset = getHeadingOffset();

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        bbb = hardwareMap.get(Servo.class, "bbb");
        bbb.setPosition(GlobalConstants.BBB_IDLE_POS);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(101.16, -160, DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
        sleep(300);
        odo.setPosition(new Pose2D(DistanceUnit.INCH, 72.0, 72.0, AngleUnit.DEGREES, 0.0));
        odo.update();

        try {
            visionLocalizer = new LimelightPinpointLocalizer(hardwareMap, "limelight");
            visionLocalizer.start();
            visionLocalizer.getLimelight().pipelineSwitch(0);
            telemetry.addLine("[OK] Limelight Localizer Ready.");
        } catch (Exception e) {
            telemetry.addLine("[ERROR] Limelight Initialisation Failed!");
            visionLocalizer = null;
        }

        driveSubsystem = new MecanumDriveSubsystem(hardwareMap);
        autoAimSubsystem = new AutoAimSubsystem(hardwareMap);
        flywheelSubsystem = new FlywheelSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        climbSubsystem = new ClimbSubsystem();
        climbSubsystem.init(hardwareMap);

        telemetry.addLine("Ready to Start - Right Stick to Calibrate XY");
        telemetry.addData("Alliance Target X", TARGET_X_WORLD);
        telemetry.addData("Alliance Target Y", TARGET_Y_WORLD);
        telemetry.addData("Heading Offset", headingOffset);
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

            double globalVx = odo.getVelX(DistanceUnit.INCH);
            double globalVy = odo.getVelY(DistanceUnit.INCH);
            boolean currentRightStickButton = gamepad1.right_stick_button;
            visionCalibrationSuccess = false;

            if (currentRightStickButton && !lastRightStickButton && visionLocalizer != null) {
                Pose2D visionPose = visionLocalizer.getTransformedPose(rawHeadingDeg);
                if (visionPose != null) {
                    double targetWorldX_Inches = visionPose.getX(DistanceUnit.INCH);
                    double targetWorldY_Inches = visionPose.getY(DistanceUnit.INCH);
                    double distToTarget = Math.hypot(TARGET_X_WORLD - rx_odo, TARGET_Y_WORLD - ry_odo);
                    double clampedDist = Math.max(20.0, Math.min(150.0, distToTarget));
                    double llWeight = 0.95 - ((clampedDist - 20.0) / (150.0 - 20.0)) * (0.95 - 0.3);
                    double odoWeight = 1.0 - llWeight;
                    double fusedX = (rx_odo * odoWeight) + (targetWorldX_Inches * llWeight);
                    double fusedY = (ry_odo * odoWeight) + (targetWorldY_Inches * llWeight);

                    odo.setPosition(new Pose2D(DistanceUnit.INCH, fusedX, fusedY, AngleUnit.DEGREES, rawHeadingDeg));
                    gamepad1.rumble(0.5, 0.5, 200);
                    visionCalibrationSuccess = true;
                    rx_odo = fusedX;
                    ry_odo = fusedY;
                }
            }
            lastRightStickButton = currentRightStickButton;

            boolean isClimbing = gamepad1.touchpad;

            boolean currentBackState = gamepad1.back;
            if (currentBackState && !lastBackState) {
                climbSubsystem.togglePto();
            }
            lastBackState = currentBackState;

            if (isClimbing) {
                climbSubsystem.runAutoClimb(true, telemetry);
            } else {
                climbSubsystem.runAutoClimb(false, telemetry);

                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x * 1.1;
                double rx_drive = gamepad1.right_stick_x;
                driveSubsystem.driveFieldCentric(x, y, rx_drive, currentHeadingDeg);
            }

            boolean currentCircleState = gamepad1.b;
            if (currentCircleState && !lastCircleState) {
                isShootingMode = !isShootingMode;
                if (isShootingMode) bbbTimer.reset();
            }
            lastCircleState = currentCircleState;

            boolean currentSquareState = gamepad1.x;
            if (currentSquareState && !lastSquareState) isShootingMode = false;
            lastSquareState = currentSquareState;

            boolean currentLeftBumperState = gamepad1.left_bumper;
            if (currentLeftBumperState && !lastLeftBumperState) isManualMode = !isManualMode;
            lastLeftBumperState = currentLeftBumperState;

            if (isManualMode) {
                if (gamepad1.dpad_left) manualTargetDistance = 25.0;
                else if (gamepad1.dpad_up) manualTargetDistance = 54.23;
                else if (gamepad1.dpad_right) manualTargetDistance = 88.0;
                else if (gamepad1.dpad_down) manualTargetDistance = 150.0;
            }

            boolean isEmergencyBrake = gamepad1.right_bumper;
            double robotOmega = odo.getHeadingVelocity(AngleUnit.DEGREES.getUnnormalized());

            AutoAimSubsystem.TurretCommand aimCommand = autoAimSubsystem.update(
                    rx_odo, ry_odo, globalVx, globalVy, rawHeadingDeg, robotOmega,
                    TARGET_X_WORLD, TARGET_Y_WORLD,
                    isManualMode, manualTargetDistance
            );

            boolean isBBBReady = !isShootingMode || (bbbTimer.milliseconds() >= GlobalConstants.BBB_DELAY_MS);
            double targetVelocityRPM = GlobalConstants.FLYWHEEL_RPM_MIN;

            if (isEmergencyBrake) {
                targetVelocityRPM = 0;
                bbb.setPosition(GlobalConstants.BBB_IDLE_POS);
            } else if (aimCommand.hasTarget) {
                targetVelocityRPM = aimCommand.targetRpm;
                if (isShootingMode) bbb.setPosition(GlobalConstants.BBB_SHOOT_POS);
                else bbb.setPosition(GlobalConstants.BBB_IDLE_POS);
            }

            flywheelSubsystem.update(targetVelocityRPM, isEmergencyBrake, aimCommand.hasTarget);

            boolean rpmOK = flywheelSubsystem.isReady();
            if (aimCommand.hasTarget && aimCommand.targetDist >= 130.0) {
                rpmOK = Math.abs(flywheelSubsystem.getCurrentRPM() - aimCommand.targetRpm) <= 114514.0;
            }

            boolean effectiveAimLocked = isManualMode ? true : aimCommand.isAimLocked;
            boolean conditionsMet = aimCommand.hasTarget && rpmOK && effectiveAimLocked;

            if (isShootingMode) {
                if (conditionsMet) {
                    long currentTime = System.currentTimeMillis();
                    if (currentTime - lastRumbleTime > 250) {
                        gamepad1.rumble(0.5, 0.5, 100);
                        lastRumbleTime = currentTime;
                    }
                }
            } else {
                if (conditionsMet && !lastConditionsMet) {
                    gamepad1.rumble(1.0, 1.0, 150);
                }
            }
            lastConditionsMet = conditionsMet;

            intakeSubsystem.update(
                    isShootingMode,
                    aimCommand.hasTarget,
                    aimCommand.isUnwinding,
                    effectiveAimLocked,
                    rpmOK,
                    aimCommand.targetDist,
                    isBBBReady,
                    flywheelSubsystem.getCurrentRPM(),
                    aimCommand.targetRpm,
                    autoAimSubsystem.getCurrentBatteryVoltage()
            );

            telemetry.addData("Mode", isManualMode ? "MANUAL" : "AUTO-AIM");
            telemetry.addData("Shooting Mode", isShootingMode ? "ACTIVE" : "IDLE");
            telemetry.addData("PTO State", climbSubsystem.isPtoUnlocked() ? "UNLOCKED" : "LOCKED");
            telemetry.addData("Calibrate State", visionCalibrationSuccess ? "SUCCESS!" : "Wait for Trigger");
            telemetry.update();
        }

        if (visionLocalizer != null) visionLocalizer.stop();
        autoAimSubsystem.stop();
        driveSubsystem.stop();
    }
}