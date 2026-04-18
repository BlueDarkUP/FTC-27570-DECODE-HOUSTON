package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private final double TARGET_Y_WORLD = 133.0;

    private boolean isShootingMode = false;
    private boolean lastCircleState = false;
    private boolean isManualMode = false;
    private boolean lastLeftBumperState = false;
    private boolean lastSquareState = false;

    private double manualTargetDistance = 25.0;
    private double headingOffset = 0.0;

    private boolean lastConditionsMet = false;
    private long lastRumbleTime = 0;

    private ElapsedTime bbbTimer = new ElapsedTime();
    private final double BBB_DELAY_MS = 300.0;

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

        telemetry.addLine("Ready to Start");
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

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx_drive = gamepad1.right_stick_x;

            if (gamepad1.right_stick_button) {
                headingOffset = rawHeadingDeg;
            }

            driveSubsystem.driveFieldCentric(x, y, rx_drive, currentHeadingDeg);

            boolean currentSquareState = gamepad1.x;
            if (currentSquareState && !lastSquareState) isShootingMode = false;
            lastSquareState = currentSquareState;

            boolean currentCircleState = gamepad1.b;
            if (currentCircleState && !lastCircleState) {
                isShootingMode = !isShootingMode;
                if (isShootingMode) bbbTimer.reset();
            }
            lastCircleState = currentCircleState;

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

            boolean isBBBReady = !isShootingMode || (bbbTimer.milliseconds() >= BBB_DELAY_MS);

            double targetVelocityRPM = FlywheelSubsystem.IDLE_VELOCITY_MIN;
            if (isEmergencyBrake) {
                targetVelocityRPM = 0;
                bbb.setPosition(0);
            } else if (aimCommand.hasTarget) {
                targetVelocityRPM = aimCommand.targetRpm;
                if (isShootingMode) bbb.setPosition(0.18);
                else bbb.setPosition(0.0);
            }

            flywheelSubsystem.update(targetVelocityRPM, isEmergencyBrake, aimCommand.hasTarget);

            boolean rpmOK = flywheelSubsystem.isReady();
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

            intakeSubsystem.update(isShootingMode, aimCommand.hasTarget, aimCommand.isUnwinding, effectiveAimLocked, rpmOK, aimCommand.targetDist, isBBBReady);

            telemetry.addData("BBB Status", isBBBReady ? "READY" : "OPENING...");
            telemetry.addData("Turret Lock", aimCommand.isAimLocked ? "LOCKED" : "TRACKING");
            telemetry.update();
        }
        autoAimSubsystem.stop();
        driveSubsystem.stop();
    }
}