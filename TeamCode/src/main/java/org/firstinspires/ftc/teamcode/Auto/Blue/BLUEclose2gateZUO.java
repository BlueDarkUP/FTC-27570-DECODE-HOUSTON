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

// 引入你的子系统包
import org.firstinspires.ftc.teamcode.Subsystems.ForAuto.*;

@Autonomous(name = "BLUEclose2gateZUO", group = "Autonomous")
public class BLUEclose2gateZUO extends OpMode {

    // 底盘跟随器
    private Follower follower;

    // 复用子系统
    private PitchSubsystem pitch;
    private TurretSubsystem turret;
    private FlywheelSubsystem flywheel;
    private IntakeShooterSubsystem intakeShooter;

    private Timer pathTimer;
    private Timer opmodeTimer;
    private Timer loopTimer; // 新增：专门用于记录开门嘬循环是否到达10秒的计时器
    private int pathState;

    // 路径声明
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

    // 起始点
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

        // ================== Path11 和 Path12 占位符 ==================
        // 请在此处填入你实际抓取的开门嘬与发射坐标
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
        // =============================================================

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
            // ================== 1. 预制并发射 ==================
            case 10:
                follower.followPath(fasheyuzhi, true);
                turret.setTargetAngle(-51.5);
                flywheel.setTargetRPM(3250.0);
                intakeShooter.setIntakePower(0.0);
                setPathState(11);
                break;
            case 11:
                if (!follower.isBusy()) {
                    setPathState(12); // 进入等待微调状态，自动重置 pathTimer
                }
                break;
            case 12:
                if (pathTimer.getElapsedTimeSeconds() >= 0.2) { // 等待0.2秒让底盘微调
                    intakeShooter.startPrecisionShoot(0.45);
                    setPathState(13);
                }
                break;
            case 13:
                if (!intakeShooter.isShootingActive()) {
                    setPathState(20);
                }
                break;

            // ================== 2. 吸第一排 ==================
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

            // ================== 3. 第一次开门 (等待1秒) ==================
            case 30:
                pitch.setPitch(0.7);
                flywheel.setTargetRPM(3400.0);
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

            // ================== 4. 射第一排 ==================
            case 40:
                follower.followPath(fashediyipai, true);
                setPathState(41);
                break;
            case 41:
                if (!follower.isBusy()) {
                    setPathState(42); // 进入等待微调状态
                }
                break;
            case 42:
                if (pathTimer.getElapsedTimeSeconds() >= 0.2) { // 等待0.2秒让底盘微调
                    intakeShooter.startPrecisionShoot(0.45);
                    setPathState(43);
                }
                break;
            case 43:
                if (!intakeShooter.isShootingActive()) {
                    setPathState(50);
                }
                break;

            // ================== 5. 吸第二排 ==================
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

            // ================== 6. 第二次开门 (等待1秒) ==================
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

            // ================== 7. 射第二排 ==================
            case 70:
                follower.followPath(fashedierpai, true);
                setPathState(71);
                break;
            case 71:
                if (!follower.isBusy()) {
                    setPathState(72); // 进入等待微调状态
                }
                break;
            case 72:
                if (pathTimer.getElapsedTimeSeconds() >= 0.2) { // 等待0.2秒让底盘微调
                    intakeShooter.startPrecisionShoot(0.45);
                    setPathState(73);
                }
                break;
            case 73:
                if (!intakeShooter.isShootingActive()) {
                    loopTimer.resetTimer();
                    setPathState(80);
                }
                break;

            // ================== 8. 持续10秒的开门嘬与发射循环 ==================
            case 80:
                // 每次准备出发去吸取前，检查循环是否已经过了 10 秒
                if (loopTimer.getElapsedTimeSeconds() >= 10.0) {
                    // 大于等于10秒，强制退出循环，去吸第三排
                    setPathState(90);
                } else {
                    // 时间未满10秒，出发去执行 Path11
                    follower.followPath(Path11, true); // holdEnd
                    intakeShooter.setIntakePower(1.0);
                    setPathState(81);
                }
                break;
            case 81:
                if (!follower.isBusy()) {
                    // 到达位置，重置常规路径计时器，准备非阻塞等待2秒
                    pathTimer.resetTimer();
                    setPathState(82);
                }
                break;
            case 82:
                // 原地嘬 2 秒
                turret.setTargetAngle(-52.5);
                if (pathTimer.getElapsedTimeSeconds() >= 1.5) {
                    setPathState(83);
                }
                break;
            case 83:
                // 执行 Path12 返回发射
                follower.followPath(Path12, true); // holdEnd
                intakeShooter.setIntakePower(0.0);
                setPathState(84);
                break;
            case 84:
                if (!follower.isBusy()) {
                    setPathState(85); // 进入等待微调状态
                }
                break;
            case 85:
                if (pathTimer.getElapsedTimeSeconds() >= 0.2) { // 等待0.2秒让底盘微调
                    intakeShooter.startPrecisionShoot(0.45);
                    setPathState(86);
                }
                break;
            case 86:
                if (!intakeShooter.isShootingActive()) {
                    // 射击完成，回到循环起点 80，重新判断时间
                    setPathState(80);
                }
                break;

            // ================== 9. 吸第三排 ==================
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

            // ================== 10. 射第三排 ==================
            case 100:
                follower.followPath(fashedisanpai, true);
                intakeShooter.setIntakePower(0.0);
                setPathState(101);
                break;
            case 101:
                if (!follower.isBusy()) {
                    setPathState(102); // 进入等待微调状态
                }
                break;
            case 102:
                if (pathTimer.getElapsedTimeSeconds() >= 0.2) { // 等待0.2秒让底盘微调
                    intakeShooter.startPrecisionShoot(0.45);
                    setPathState(103);
                }
                break;
            case 103:
                if (!intakeShooter.isShootingActive()) {
                    setPathState(110);
                }
                break;

            // ================== 11. 最终停靠 ==================
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

            // ================== 结束 ==================
            case 120:
                flywheel.setTargetRPM(0.0);
                intakeShooter.setIntakePower(0.0);
                intakeShooter.setBBServo(0.0);
                setPathState(-1);
                break;
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        loopTimer = new Timer(); // 初始化循环计时器

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
        pitch.setPitch(1.0);
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
        pitch.setPitch(1.0);

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