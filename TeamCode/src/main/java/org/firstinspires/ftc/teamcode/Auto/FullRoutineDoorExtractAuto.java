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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Full Routine with Door Extract Loop", group = "Autonomous")
public class FullRoutineDoorExtractAuto extends OpMode {

    private Follower follower;
    private DcMotorEx intakeMotor;
    private DcMotorEx turretMotor;

    private DcMotorEx motorSH;
    private DcMotorEx motorHS;
    private Servo bbb;

    private Servo LP;
    private Servo RP;

    private Timer pathTimer;
    private Timer opmodeTimer;
    private int pathState;

    private final int MAX_DOOR_EXTRACT_LOOPS = 2;
    private int doorExtractLoopCount = 0;

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

    private double turretTargetAngle = 0.0;

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

    private double flywheelTargetRPM = 0.0;
    private double flywheelCurrentRPM = 0.0;

    private enum ShootMode { NONE, PRECISION, BLIND }
    private ShootMode currentShootMode = ShootMode.NONE;
    private ElapsedTime shootTimer;
    private double activeShootDuration = 0.0;

    private final double LP_UP = 1;
    private final double LP_DOWN = 0.4;
    private final double RP_UP = 0;
    private final double RP_DOWN = 0.5;

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
                                new Pose(12.3, 59.8)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(226), Math.toRadians(150))
                .build();

        fashekaimenzuo = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(12.3, 59.8),
                                new Pose(35.000, 47.000),
                                new Pose(60.000, 83.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(226))
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
                                new Pose(12.000, 36.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        fashedisanpai = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(12.000, 36.000),
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

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 10:
                follower.setMaxPower(0.7);
                follower.followPath(diyigepaoda, false);
                turretTargetAngle = -48.0;
                flywheelTargetRPM = 4000.0;
                intakeMotor.setPower(0.0);
                setPathState(11);
                break;
            case 11:
                if (pathTimer.getElapsedTimeSeconds() >= 1.2) {
                    startBlindShoot(0.5);
                    setPathState(12);
                }
                break;
            case 12:
                if (!isShootingActive()) {
                    follower.setMaxPower(1.0);
                    setPathState(13);
                }
                else if (!follower.isBusy()) {
                    follower.setMaxPower(1.0);
                    setPathState(20);
                }
                break;
            case 13:
                if (!follower.isBusy()) {
                    setPathState(20);
                }
                break;

            case 20:
                follower.setMaxPower(1);
                follower.followPath(xidierpai, false);
                bbb.setPosition(0.0);
                intakeMotor.setPower(1.0);
                flywheelTargetRPM = 3250;
                setPathState(21);
                break;
            case 21:
                if (!follower.isBusy()) { setPathState(30); }
                break;

            case 30:
                follower.followPath(fashedierpai, true);
                turretTargetAngle = -93;
                flywheelTargetRPM = 3250.0;
                intakeMotor.setPower(0.0);
                setPathState(31);
                break;
            case 31:
                if (!follower.isBusy()) {
                    setPathState(315);
                }
                break;
            case 315:
                if (pathTimer.getElapsedTimeSeconds() >= 0.4) {
                    startPrecisionShoot(0.6);
                    setPathState(32);
                }
                break;
            case 32:
                if (!isShootingActive()) {
                    doorExtractLoopCount = 0;
                    setPathState(40);
                }
                break;

            case 40:
                follower.followPath(kaimenzuo, true);
                intakeMotor.setPower(1.0);
                flywheelTargetRPM = 3250.0;
                setPathState(41);
                break;
            case 41:
                if (!follower.isBusy()) {
                    setPathState(42);
                }
                break;
            case 42:
                if (pathTimer.getElapsedTimeSeconds() >= 2) {
                    setPathState(50);
                }
                break;

            case 50:
                follower.followPath(fashekaimenzuo, true);
                turretTargetAngle = -93;
                flywheelTargetRPM = 3250.0;
                intakeMotor.setPower(1.0);
                setPathState(51);
                break;
            case 51:
                if (!follower.isBusy()) {
                    setPathState(515);
                }
                break;
            case 515:
                if (pathTimer.getElapsedTimeSeconds() >= 0.4) {
                    startPrecisionShoot(0.6);
                    setPathState(52);
                }
                break;
            case 52:
                if (!isShootingActive()) {
                    doorExtractLoopCount++;

                    if (doorExtractLoopCount < MAX_DOOR_EXTRACT_LOOPS) {
                        setPathState(40);
                    } else {
                        setPathState(60);
                    }
                }
                break;

            case 60:
                follower.followPath(zhunbeixidiyipai, false);
                flywheelTargetRPM = 3250.0;
                intakeMotor.setPower(1.0);
                setPathState(61);
                break;
            case 61:
                if (!follower.isBusy() || follower.atParametricEnd()) {
                    setPathState(70);
                }
                break;

            case 70:
                follower.followPath(xidiyipai, false);
                intakeMotor.setPower(1.0);
                setPathState(71);
                break;
            case 71:
                if (!follower.isBusy()) { setPathState(80); }
                break;

            case 80:
                follower.followPath(fashediyipai, true);
                turretTargetAngle = -93;
                flywheelTargetRPM = 3250.0;
                intakeMotor.setPower(0.0);
                setPathState(81);
                break;
            case 81:
                if (!follower.isBusy()) {
                    setPathState(815);
                }
                break;
            case 815:
                if (pathTimer.getElapsedTimeSeconds() >= 0.4) {
                    startPrecisionShoot(0.6);
                    setPathState(82);
                }
                break;
            case 82:
                if (!isShootingActive()) { setPathState(90); }
                break;

            case 90:
                follower.followPath(zhunbeixidisanpai, false);
                flywheelTargetRPM = 3250.0;
                intakeMotor.setPower(1.0);
                setPathState(91);
                break;
            case 91:
                if (!follower.isBusy() || follower.atParametricEnd()) {
                    setPathState(100);
                }
                break;

            case 100:
                follower.followPath(xidisanpai, false);
                intakeMotor.setPower(1.0);
                setPathState(101);
                break;
            case 101:
                if (!follower.isBusy()) { setPathState(110); }
                break;

            case 110:
                follower.followPath(fashedisanpai, true);
                turretTargetAngle = -113.0;
                flywheelTargetRPM = 3250.0;
                intakeMotor.setPower(0.0);
                setPathState(111);
                break;
            case 111:
                if (!follower.isBusy()) {
                    setPathState(1115);
                }
                break;
            case 1115:
                if (pathTimer.getElapsedTimeSeconds() >= 0.4) {
                    startPrecisionShoot(0.6);
                    setPathState(112);
                }
                break;
            case 112:
                if (!isShootingActive()) { setPathState(120); }
                break;

            case 120:
                follower.followPath(tingkao, true);
                turretTargetAngle = 0.0;
                flywheelTargetRPM = 0.0;
                intakeMotor.setPower(0.0);
                bbb.setPosition(0.0);
                setPathState(121);
                break;
            case 121:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        pidTimer = new ElapsedTime();
        flywheelTimer = new ElapsedTime();
        shootTimer = new ElapsedTime();

        doorExtractLoopCount = 0;
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
        setPitchServos(0.7);
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
        setPitchServos(0.87);

        follower.update();
        autonomousPathUpdate();

        updateTurret();
        updateFlywheel();

        updateShootingSequence();

        telemetry.addData("Path State", pathState);
        telemetry.addData("Shoot Mode", currentShootMode);
        telemetry.addData("RPM Current/Target", "%.1f / %.1f", flywheelCurrentRPM, flywheelTargetRPM);
        telemetry.addData("Door Loop Progress", "%d / %d", doorExtractLoopCount, MAX_DOOR_EXTRACT_LOOPS);
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