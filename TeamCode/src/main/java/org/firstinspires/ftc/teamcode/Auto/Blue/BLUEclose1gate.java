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

@Autonomous(name = "蓝方开一次门", group = "Autonomous")
public class BLUEclose1gate extends OpMode {

    // 底盘跟随器
    private Follower follower;

    // 复用子系统
    private PitchSubsystem pitch;
    private TurretSubsystem turret;
    private FlywheelSubsystem flywheel;
    private IntakeShooterSubsystem intakeShooter;

    private Timer pathTimer;
    private Timer opmodeTimer;
    private int pathState;

    // 路径声明
    public PathChain fasheyuzhi;
    public PathChain xidiyipai;
    public PathChain kaimen;
    public PathChain fashediyipai;
    public PathChain xidierpai;
    public PathChain fashedierpai;
    public PathChain xidisanpai;
    public PathChain fashedisanpai;
    public PathChain tingkao;

    // 起始点基于你提供的新路线第一段
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

        xidierpai = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.000, 75.000),
                                new Pose(65.000, 58.000),
                                new Pose(13.000, 58.500)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        fashedierpai = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(13.000, 58.500),
                                new Pose(39.500, 62.750),
                                new Pose(60.000, 75.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
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
            // ================== 阶段 1：起步并发射预置 ==================
            case 10:
                follower.followPath(fasheyuzhi, true);
                turret.setTargetAngle(-53.0);    // 起步云台 -56
                flywheel.setTargetRPM(3250.0);   // 全程转速 3250
                intakeShooter.setIntakePower(0.0);
                setPathState(11);
                break;
            case 11:
                // 到位后不等待，直接开火
                if (!follower.isBusy()) {
                    intakeShooter.startPrecisionShoot(0.5);
                    setPathState(12);
                }
                break;
            case 12:
                // 等待0.5秒发射动作结束
                if (!intakeShooter.isShootingActive()) {
                    setPathState(20);
                }
                break;

            // ================== 阶段 2：吸取第一排 ==================
            case 20:
                pitch.setPitch(0.75);
                flywheel.setTargetRPM(3350.0);
                follower.followPath(xidiyipai, false);
                intakeShooter.setIntakePower(1.0); // 打开 Intake
                setPathState(21);
                break;
            case 21:
                if (!follower.isBusy()) {
                    setPathState(30);
                }
                break;

            // ================== 阶段 3：停 Intake，开门 ==================
            case 30:
                follower.followPath(kaimen, true); // 确保是 true (holdEnd)
                intakeShooter.setIntakePower(0.0);
                turret.setTargetAngle(-50.0);
                setPathState(31);
                break;
            case 31:
                if (!follower.isBusy()) {
                    // 新增：到位后先不要立刻跳到下一条路，而是重置计时器
                    pathTimer.resetTimer();
                    setPathState(32);
                }
                break;
            case 32:
                // 新增：非阻塞等待1秒
                if (pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    setPathState(40); // 1秒后才进入发球路线
                }
                break;

            // ================== 阶段 4：回发第一排 ==================
            case 40:
                follower.followPath(fashediyipai, true);
                setPathState(41);
                break;
            case 41:
                // 到位后不等待，直接开火
                if (!follower.isBusy()) {
                    intakeShooter.startPrecisionShoot(0.5);
                    setPathState(42);
                }
                break;
            case 42:
                if (!intakeShooter.isShootingActive()) {
                    setPathState(50);
                }
                break;

            // ================== 阶段 5：吸第二排 ==================
            case 50:
                follower.followPath(xidierpai, false);
                intakeShooter.setIntakePower(1.0); // 打开 Intake
                setPathState(51);
                break;
            case 51:
                if (!follower.isBusy()) {
                    setPathState(60);
                }
                break;

            // ================== 阶段 6：回发第二排 ==================
            case 60:
                follower.followPath(fashedierpai, true);
                intakeShooter.setIntakePower(0.0); // 发射前最好关掉或根据需要保留，此处选择关闭
                setPathState(61);
                break;
            case 61:
                // 到位后不等待，直接开火
                if (!follower.isBusy()) {
                    intakeShooter.startPrecisionShoot(0.5);
                    setPathState(62);
                }
                break;
            case 62:
                if (!intakeShooter.isShootingActive()) {
                    setPathState(70);
                }
                break;

            // ================== 阶段 7：吸第三排 ==================
            case 70:
                follower.followPath(xidisanpai, false);
                intakeShooter.setIntakePower(1.0); // 打开 Intake
                setPathState(71);
                break;
            case 71:
                if (!follower.isBusy()) {
                    setPathState(80);
                }
                break;

            // ================== 阶段 8：回发第三排 ==================
            case 80:
                follower.followPath(fashedisanpai, true);
                intakeShooter.setIntakePower(0.0);
                setPathState(81);
                break;
            case 81:
                // 到位后不等待，直接开火
                if (!follower.isBusy()) {
                    intakeShooter.startPrecisionShoot(0.5);
                    setPathState(82);
                }
                break;
            case 82:
                if (!intakeShooter.isShootingActive()) {
                    setPathState(90);
                }
                break;

            // ================== 阶段 9：最终停靠 ==================
            case 90:
                follower.followPath(tingkao, true);
                turret.setTargetAngle(0.0); // 停靠云台回到角度 0
                setPathState(91);
                break;
            case 91:
                if (!follower.isBusy()) {
                    setPathState(100);
                }
                break;

            // ================== 阶段 10：程序结束 ==================
            case 100:
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

        // 初始化各个子系统
        pitch = new PitchSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);
        flywheel = new FlywheelSubsystem(hardwareMap);
        intakeShooter = new IntakeShooterSubsystem(hardwareMap);

        // 初始化底盘与路径
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {
        pitch.setPitch(1.0); // 俯仰1
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
        // 整个自动阶段维持俯仰1
        pitch.setPitch(1.0);

        follower.update();
        autonomousPathUpdate();

        turret.update();
        flywheel.update();
        intakeShooter.update(flywheel);

        telemetry.addData("Path State", pathState);
        telemetry.addData("Shoot Mode", intakeShooter.getCurrentShootMode());
        telemetry.addData("Turret Target Angle", turret.getTargetAngle());
        telemetry.addData("RPM Current/Target", "%.1f / %.1f", flywheel.getCurrentRPM(), flywheel.getTargetRPM());
        telemetry.update();
    }

    @Override
    public void stop() {
        turret.stop();
        flywheel.stop();
        intakeShooter.stop();
    }
}