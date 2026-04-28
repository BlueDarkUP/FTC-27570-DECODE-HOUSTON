package org.firstinspires.ftc.teamcode.Auto.Red;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Storage.RobotStateStorage;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.GlobalConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ForAuto.*;

@Autonomous(name = "红方开门嘬两次自动", group = "Autonomous")
public class REDclose2gateZUO extends OpMode {

    private Follower follower;
    private PitchSubsystem pitch;
    private TurretSubsystem turret;
    private FlywheelSubsystem flywheel;
    private IntakeShooterSubsystem intakeShooter;
    private DcMotorEx rawTurretMotor;

    private Timer pathTimer;
    private Timer opmodeTimer;
    private Timer loopTimer;
    private int pathState;

    public PathChain fasheyuzhi;
    public PathChain xidiyipai;
    public PathChain kaimen;
    public PathChain fashediyipai;
    public PathChain kaidiercimen;
    public PathChain Path10;
    public PathChain fashedierpai;
    public PathChain Path11;
    public PathChain Path12;
    public PathChain xidisanpai;
    public PathChain fashedisanpai;
    public PathChain tingkao;

    private final Pose startPose = new Pose(109.000, 135.000, Math.toRadians(0));

    public void buildPaths() {
        fasheyuzhi = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(109.000, 135.000),
                                new Pose(100.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        xidiyipai = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(100.000, 84.000),
                                new Pose(126.000, 84.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        kaimen = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(126.000, 84.000),
                                new Pose(123.692, 75.308),
                                new Pose(127.000, 75.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        fashediyipai = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(127.000, 75.000),
                                new Pose(84.000, 75.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        kaidiercimen = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(84.000, 75.000),
                                new Pose(79.000, 58.000),
                                new Pose(131.000, 58.500)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Path10 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(131.000, 58.500),
                                new Pose(108.000, 61.000),
                                new Pose(125.000, 68.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        fashedierpai = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(126.000, 65.077),
                                new Pose(84.000, 75.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        Path11 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(84.000, 75.000),
                                new Pose(102.000, 60.000),
                                new Pose(132.700, 60)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(23))
                .build();

        Path12 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(132.700, 60),
                                new Pose(99.000, 61.300),
                                new Pose(84.000, 75.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(23), Math.toRadians(0))
                .build();

        xidisanpai = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(84.000, 75.000),
                                new Pose(79.000, 30.000),
                                new Pose(131.000, 35.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        fashedisanpai = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(131.000, 35.000),
                                new Pose(84.000, 75.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        tingkao = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(84.000, 75.000),
                                new Pose(84.000, 60.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 10:
                follower.followPath(fasheyuzhi, true);
                turret.setTargetAngle(50.5);
                flywheel.setTargetRPM(GlobalConstants.AUTO_RPM_NORMAL);
                intakeShooter.setIntakePower(0.0);
                setPathState(11);
                break;
            case 11:
                if (!follower.isBusy()) {
                    setPathState(12);
                }
                break;
            case 12:
                if (pathTimer.getElapsedTimeSeconds() >= 0.1) {
                    intakeShooter.startPrecisionShoot(GlobalConstants.SHOOT_TIME_SHORT);
                    setPathState(13);
                }
                break;
            case 13:
                if (!intakeShooter.isShootingActive()) {
                    setPathState(20);
                }
                break;

            case 20:
                follower.followPath(xidiyipai, false);
                intakeShooter.setIntakePower(1.0);
                setPathState(21);
                break;
            case 21:
                if (!follower.isBusy()) {
                    setPathState(30);
                }
                break;

            case 30:
                pitch.setPitch(GlobalConstants.PITCH_POS_INTAKE_DEEP);
                flywheel.setTargetRPM(GlobalConstants.AUTO_RPM_DOOR_2);
                follower.followPath(kaimen, false);
                intakeShooter.setIntakePower(0.0);
                turret.setTargetAngle(50.3);
                setPathState(31);
                break;
            case 31:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(32);
                }
                break;
            case 32:
                if (pathTimer.getElapsedTimeSeconds() >= 0.1) {
                    setPathState(40);
                }
                break;

            case 40:
                follower.followPath(fashediyipai, true);
                setPathState(41);
                break;
            case 41:
                if (!follower.isBusy()) {
                    setPathState(42);
                }
                break;
            case 42:
                if (pathTimer.getElapsedTimeSeconds() >= 0.1) {
                    intakeShooter.startPrecisionShoot(GlobalConstants.SHOOT_TIME_SHORT);
                    setPathState(43);
                }
                break;
            case 43:
                if (!intakeShooter.isShootingActive()) {
                    setPathState(50);
                }
                break;

            case 50:
                follower.followPath(kaidiercimen, false);
                intakeShooter.setIntakePower(1.0);
                setPathState(51);
                break;
            case 51:
                if (!follower.isBusy()) {
                    setPathState(60);
                }
                break;

            case 60:
                follower.followPath(Path10, false);
                intakeShooter.setIntakePower(0.0);
                setPathState(61);
                break;
            case 61:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(62);
                }
                break;
            case 62:
                if (pathTimer.getElapsedTimeSeconds() >= 0.1) {
                    setPathState(70);
                }
                break;

            case 70:
                follower.followPath(fashedierpai, true);
                setPathState(71);
                break;
            case 71:
                if (!follower.isBusy()) {
                    setPathState(72);
                }
                break;
            case 72:
                if (pathTimer.getElapsedTimeSeconds() >= 0.1) {
                    intakeShooter.startPrecisionShoot(GlobalConstants.SHOOT_TIME_SHORT);
                    setPathState(73);
                }
                break;
            case 73:
                if (!intakeShooter.isShootingActive()) {
                    loopTimer.resetTimer();
                    setPathState(80);
                }
                break;

            case 80:
                if (loopTimer.getElapsedTimeSeconds() >= 10.0) {
                    setPathState(90);
                } else {
                    follower.followPath(Path11, true);
                    intakeShooter.setIntakePower(1.0);
                    setPathState(81);
                }
                break;
            case 81:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(82);
                }
                break;
            case 82:
                turret.setTargetAngle(50.3);
                if (pathTimer.getElapsedTimeSeconds() >= 1.5) {
                    setPathState(83);
                }
                break;
            case 83:
                follower.followPath(Path12, true);
                intakeShooter.setIntakePower(0.0);
                setPathState(84);
                break;
            case 84:
                if (!follower.isBusy()) {
                    setPathState(85);
                }
                break;
            case 85:
                if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                    intakeShooter.startPrecisionShoot(GlobalConstants.SHOOT_TIME_SHORT);
                    setPathState(86);
                }
                break;
            case 86:
                if (!intakeShooter.isShootingActive()) {
                    setPathState(80);
                }
                break;

            case 90:
                turret.setTargetAngle(50.3);
                follower.followPath(xidisanpai, false);
                intakeShooter.setIntakePower(1.0);
                setPathState(91);
                break;
            case 91:
                if (!follower.isBusy()) {
                    setPathState(100);
                }
                break;

            case 100:
                follower.followPath(fashedisanpai, true);
                intakeShooter.setIntakePower(0.0);
                setPathState(101);
                break;
            case 101:
                if (!follower.isBusy()) {
                    setPathState(102);
                }
                break;
            case 102:
                if (pathTimer.getElapsedTimeSeconds() >= 0.1) {
                    intakeShooter.startPrecisionShoot(GlobalConstants.SHOOT_TIME_SHORT);
                    setPathState(103);
                }
                break;
            case 103:
                if (!intakeShooter.isShootingActive()) {
                    setPathState(110);
                }
                break;

            case 110:
                follower.followPath(tingkao, true);
                turret.setTargetAngle(0.0);
                setPathState(111);
                break;
            case 111:
                if (!follower.isBusy()) {
                    setPathState(120);
                }
                break;

            case 120:
                flywheel.setTargetRPM(0.0);
                intakeShooter.setIntakePower(0.0);
                intakeShooter.setBBServo(GlobalConstants.BBB_IDLE_POS);
                setPathState(-1);
                break;
        }
    }

    @Override
    public void init() {
        RobotStateStorage.clear();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        loopTimer = new Timer();
        pitch = new PitchSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);
        flywheel = new FlywheelSubsystem(hardwareMap);
        intakeShooter = new IntakeShooterSubsystem(hardwareMap);
        rawTurretMotor = hardwareMap.get(DcMotorEx.class, "Turret");
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {
        pitch.setPitch(GlobalConstants.PITCH_POS_DEFAULT);
        turret.update();
        flywheel.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        turret.resetTimer();
        flywheel.resetTimer();
        setPathState(10);
    }

    @Override
    public void loop() {
        pitch.setPitch(GlobalConstants.PITCH_POS_DEFAULT);
        follower.update();
        autonomousPathUpdate();
        turret.update();
        flywheel.update();
        intakeShooter.update(flywheel);
        telemetry.addData("Path State", pathState);
        telemetry.addData("Shoot Mode", intakeShooter.getCurrentShootMode());
        telemetry.addData("Turret Angle", turret.getTargetAngle());
        telemetry.addData("Loop Timer (s)", loopTimer.getElapsedTimeSeconds());
        telemetry.update();
        if (rawTurretMotor != null) {
            RobotStateStorage.turretAngleDeg = (rawTurretMotor.getCurrentPosition() / GlobalConstants.TURRET_TICKS_PER_REV) * 360.0;
        }
        RobotStateStorage.isAutoDataValid = true;
    }

    @Override
    public void stop() {
        turret.stop();
        flywheel.stop();
        intakeShooter.stop();
    }
}