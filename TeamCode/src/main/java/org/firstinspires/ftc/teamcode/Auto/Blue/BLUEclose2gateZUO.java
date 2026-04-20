package org.firstinspires.ftc.teamcode.Auto.Blue;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.GlobalConstants;
import org.firstinspires.ftc.teamcode.Subsystems.ForAuto.*;

@Autonomous(name = "BLUEclose2gateZUO", group = "Autonomous")
public class BLUEclose2gateZUO extends OpMode {

    private Follower follower;
    private PitchSubsystem pitch;
    private TurretSubsystem turret;
    private FlywheelSubsystem flywheel;
    private IntakeShooterSubsystem intakeShooter;

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

    private final Pose startPose = new Pose(35.000, 135.000, Math.toRadians(180));

    public void buildPaths() {
        fasheyuzhi = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(35.000, 135.000),
                                new Pose(44.000, 84.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        xidiyipai = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(44.000, 84.000),
                                new Pose(18.000, 84.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        kaimen = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(18.000, 84.000),
                                new Pose(20.308, 75.308),
                                new Pose(17.000, 75.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        fashediyipai = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(17.000, 75.000),
                                new Pose(60.000, 75.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        kaidiercimen = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.000, 75.000),
                                new Pose(65.000, 58.000),
                                new Pose(13.000, 58.500)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path10 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(13.000, 58.500),
                                new Pose(36, 61),
                                new Pose(19, 68)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        fashedierpai = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(18, 65.077),
                                new Pose(60.000, 75.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        Path11 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(49.000, 83.000),
                                new Pose(45.000, 61.3000),
                                new Pose(11.3, 61.4)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(157))
                .build();

        Path12 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(11.3, 61),
                                new Pose(45.000, 61.3000),
                                new Pose(60.000, 75.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(157), Math.toRadians(180))
                .build();

        xidisanpai = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.000, 75.000),
                                new Pose(65.000, 30.000),
                                new Pose(13.000, 35.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        fashedisanpai = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(13.000, 35.000),
                                new Pose(60.000, 75.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        tingkao = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(60.000, 75.000),
                                new Pose(60.000, 60.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
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
                turret.setTargetAngle(-51.5);
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
                if (pathTimer.getElapsedTimeSeconds() >= 0.2) {
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
                turret.setTargetAngle(-50.3);
                setPathState(31);
                break;
            case 31:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(32);
                }
                break;
            case 32:
                if (pathTimer.getElapsedTimeSeconds() >= 0.2) {
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
                if (pathTimer.getElapsedTimeSeconds() >= 0.2) {
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
                if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
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
                if (pathTimer.getElapsedTimeSeconds() >= 0.2) {
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
                turret.setTargetAngle(-52.5);
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
                if (pathTimer.getElapsedTimeSeconds() >= 0.2) {
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
                turret.setTargetAngle(-50.3);
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
                if (pathTimer.getElapsedTimeSeconds() >= 0.2) {
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
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        loopTimer = new Timer();
        pitch = new PitchSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);
        flywheel = new FlywheelSubsystem(hardwareMap);
        intakeShooter = new IntakeShooterSubsystem(hardwareMap);
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
    }

    @Override
    public void stop() {
        turret.stop();
        flywheel.stop();
        intakeShooter.stop();
    }
}