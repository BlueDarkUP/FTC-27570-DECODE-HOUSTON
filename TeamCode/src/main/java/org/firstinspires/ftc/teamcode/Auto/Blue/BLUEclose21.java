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

@Autonomous(name = "近点21球自动", group = "Autonomous")
public class BLUEclose21 extends OpMode {

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

    public PathChain fasheyuzhi;
    public PathChain xidierpai;
    public PathChain huiqufashe1;
    public PathChain kaimenzuo1;
    public PathChain huiqufashe2;
    public PathChain kaimenzuo2;
    public PathChain huiqufashe3;
    public PathChain xidiyipai;
    public PathChain huiqufashe4;
    public PathChain kaimenzuo3;
    public PathChain huiqufashe5;
    public PathChain kaimenzuo4;
    public PathChain huiqufashe6;

    // 起始点基于你提供的新路线第一段
    private final Pose startPose = new Pose(35.000, 135.000, Math.toRadians(180));

    public void buildPaths() {
        fasheyuzhi = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(35.000, 135.000),
                                new Pose(48.000, 86.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        xidierpai = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(48.000, 86.000),
                                new Pose(60.654, 54.923),
                                new Pose(13.00, 59)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        huiqufashe1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(13.00, 58.30),
                                new Pose(45.000, 61.3000),
                                new Pose(50.000, 80.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        //TODO:TTT
        double kaimenzuox = 11.5;
        double kaimenzuoy = 61.5;

        kaimenzuo1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(50.000, 80.000),
                                new Pose(45.000, 61.3000),
                                new Pose(kaimenzuox, kaimenzuoy)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(157))
                .build();

        huiqufashe2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(kaimenzuox, kaimenzuoy),
                                new Pose(45.000, 61.3000),
                                new Pose(49.000, 83.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(157), Math.toRadians(180))
                .build();

        kaimenzuo2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(49.000, 83.000),
                                new Pose(45.000, 61.3000),
                                new Pose(kaimenzuox, kaimenzuoy)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(157))
                .build();

        huiqufashe3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(kaimenzuox, kaimenzuoy),
                                new Pose(45.000, 61.3000),
                                new Pose(49.000, 83.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(157), Math.toRadians(180))
                .build();

        xidiyipai = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(49.000, 83.000),
                                new Pose(21.000, 83.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        huiqufashe4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(19.000, 83.000),
                                new Pose(50.000, 80.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        kaimenzuo3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(50.000, 80.000),
                                new Pose(45.000, 61.3000),
                                new Pose(kaimenzuox, kaimenzuoy)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(157))
                .build();

        huiqufashe5 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(kaimenzuox, kaimenzuoy),
                                new Pose(45.000, 61.3000),
                                new Pose(49.000, 83.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(157), Math.toRadians(180))
                .build();

        kaimenzuo4 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(49.000, 83.000),
                                new Pose(45.000, 61.3000),
                                new Pose(kaimenzuox, kaimenzuoy)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(157))
                .build();

        huiqufashe6 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(kaimenzuox, kaimenzuoy),
                                new Pose(45.000, 61.3000),
                                new Pose(58, 100.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(157), Math.toRadians(135))
                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            // ================== 第1阶段：停靠并发射 ==================
            case 10:
                // 修改：将跑打(false)改为停靠(true)
                flywheel.setTargetRPM(3250);
                follower.followPath(fasheyuzhi, true);
                turret.setTargetAngle(-48.0);
                // 修改：目标转速设为3600
                intakeShooter.setIntakePower(0.0);
                setPathState(11);
                break;
            case 11:
                // 等待底盘跑到路径末尾停靠
                if (!follower.isBusy()) {
                    setPathState(115);
                }
                break;
            case 115:
                // 停靠后给极短的缓冲时间（0.2秒，与其他发射逻辑保持一致），然后射击0.5秒
                if (pathTimer.getElapsedTimeSeconds() >= 0) {
                    // 使用精射模式（停靠射击）发射0.5秒
                    intakeShooter.startPrecisionShoot(0.5);
                    setPathState(12);
                }
                break;
            case 12:
                // 严格等待射击动作完全结束
                if (!intakeShooter.isShootingActive()) {
                    follower.setMaxPower(1.0);
                    // 射击完成且底盘已经到位，直接进入第2阶段
                    setPathState(20);
                }
                break;

            // ================== 第2阶段：吸第二排 ==================
            case 20:
                pitch.setPitch(0.75);
                follower.followPath(xidierpai, false);
                intakeShooter.setBBServo(0.0);
                intakeShooter.setIntakePower(1.0);
                flywheel.setTargetRPM(3250.0); // 提前准备后续RPM
                setPathState(21);
                break;
            case 21:
                if (!follower.isBusy()) { setPathState(30); }
                break;

            // ================== 第3阶段：发射（吸完第二排后）==================
            case 30:
                follower.setMaxPower(1);
                follower.followPath(huiqufashe1, true);
                turret.setTargetAngle(-53.0);
                flywheel.setTargetRPM(3250.0);
                intakeShooter.setIntakePower(0.0);
                setPathState(31);
                break;
            case 31:
                if (!follower.isBusy()) {
                    setPathState(315);
                }
                break;
            case 315:
                if (pathTimer.getElapsedTimeSeconds() >= 0) {
                    intakeShooter.startPrecisionShoot(0.5);
                    setPathState(32);
                }
                break;
            case 32:
                if (!intakeShooter.isShootingActive()) { setPathState(40); }
                break;

            case 40:
                follower.followPath(kaimenzuo1, true);
                intakeShooter.setIntakePower(1.0);
                flywheel.setTargetRPM(3250.0);
                setPathState(41);
                break;
            case 41:
                if (!follower.isBusy()) {
                    setPathState(42);
                }
                break;
            case 42:
                if (pathTimer.getElapsedTimeSeconds() >= 1) {
                    setPathState(50);
                }
                break;

            // ================== 第5阶段：发射（第1次开门后）==================
            case 50:
                follower.followPath(huiqufashe2, true);
                turret.setTargetAngle(-53.0);
                flywheel.setTargetRPM(3250.0);
                intakeShooter.setIntakePower(1.0); // 发射同时可以保持吸
                setPathState(51);
                break;
            case 51:
                if (!follower.isBusy()) {
                    setPathState(515);
                }
                break;
            case 515:
                if (pathTimer.getElapsedTimeSeconds() >= 0) {
                    intakeShooter.startPrecisionShoot(0.50);
                    setPathState(52);
                }
                break;
            case 52:
                if (!intakeShooter.isShootingActive()) { setPathState(60); }
                break;

            case 60:
                follower.followPath(kaimenzuo2, true);
                intakeShooter.setIntakePower(1.0);
                flywheel.setTargetRPM(3250.0);
                setPathState(61);
                break;
            case 61:
                if (!follower.isBusy()) {
                    setPathState(62);
                }
                break;
            case 62:
                if (pathTimer.getElapsedTimeSeconds() >= 2.3) {
                    setPathState(70);
                }
                break;

            // ================== 第7阶段：发射（第2次开门后）==================
            case 70:
                follower.followPath(huiqufashe3, true);
                turret.setTargetAngle(-53.0);
                flywheel.setTargetRPM(3250.0);
                intakeShooter.setIntakePower(1.0);
                setPathState(71);
                break;
            case 71:
                if (!follower.isBusy()) {
                    setPathState(715);
                }
                break;
            case 715:
                if (pathTimer.getElapsedTimeSeconds() >= 0) {
                    intakeShooter.startPrecisionShoot(0.50);
                    setPathState(72);
                }
                break;
            case 72:
                if (!intakeShooter.isShootingActive()) { setPathState(80); }
                break;

            // ================== 第8阶段：吸第一排 ==================
            case 80:
                follower.followPath(xidiyipai, false);
                intakeShooter.setIntakePower(1.0);
                flywheel.setTargetRPM(3250.0);
                setPathState(81);
                break;
            case 81:
                if (!follower.isBusy()) { setPathState(90); }
                break;

            // ================== 第9阶段：发射（吸完第一排后）==================
            case 90:
                follower.followPath(huiqufashe4, true);
                turret.setTargetAngle(-53.0);
                flywheel.setTargetRPM(3250.0);
                intakeShooter.setIntakePower(0.0);
                setPathState(91);
                break;
            case 91:
                if (!follower.isBusy()) {
                    setPathState(915);
                }
                break;
            case 915:
                if (pathTimer.getElapsedTimeSeconds() >= 0) {
                    intakeShooter.startPrecisionShoot(0.5);
                    setPathState(92);
                }
                break;
            case 92:
                if (!intakeShooter.isShootingActive()) { setPathState(100); }
                break;

            case 100:
                follower.followPath(kaimenzuo3, true);
                intakeShooter.setIntakePower(1.0);
                flywheel.setTargetRPM(3250.0);
                setPathState(101);
                break;
            case 101:
                if (!follower.isBusy()) {
                    setPathState(102);
                }
                break;
            case 102:
                if (pathTimer.getElapsedTimeSeconds() >= 1) {
                    setPathState(110);
                }
                break;

            // ================== 第11阶段：发射（第3次开门后）==================
            case 110:
                follower.followPath(huiqufashe5, true);
                turret.setTargetAngle(-53.0);
                flywheel.setTargetRPM(3250.0);
                intakeShooter.setIntakePower(1.0);
                setPathState(111);
                break;
            case 111:
                if (!follower.isBusy()) {
                    setPathState(1115);
                }
                break;
            case 1115:
                if (pathTimer.getElapsedTimeSeconds() >= 0) {
                    intakeShooter.startPrecisionShoot(0.50);
                    setPathState(112);
                }
                break;
            case 112:
                if (!intakeShooter.isShootingActive()) { setPathState(120); }
                break;

            case 120:
                follower.followPath(kaimenzuo4, true);
                intakeShooter.setIntakePower(1.0);

                flywheel.setTargetRPM(3250.0);

                setPathState(121);
                break;
            case 121:
                if (!follower.isBusy()) {
                    setPathState(122);
                }
                break;
            case 122:
                if (pathTimer.getElapsedTimeSeconds() >= 2) {
                    setPathState(130);
                }
                break;

            // ================== 第13阶段：最后一次停靠并发射 ==================
            case 130:
                follower.followPath(huiqufashe6, true);
                turret.setTargetAngle(15); // 只有最后一次云台角度是 -27.0
                flywheel.setTargetRPM(3250.0);
                intakeShooter.setIntakePower(0.0);
                setPathState(131);
                break;
            case 131:
                if (!follower.isBusy()) {
                    setPathState(1315);
                }
                break;
            case 1315:
                if (pathTimer.getElapsedTimeSeconds() >= 0.1) {
                    intakeShooter.startPrecisionShoot(0.5);
                    setPathState(132);
                }
                break;
            case 132:
                if (!intakeShooter.isShootingActive()) { setPathState(140); }
                break;

            // ================== 结束阶段：归零 ==================
            case 140:
                turret.setTargetAngle(0.0);
                flywheel.setTargetRPM(0.0);
                intakeShooter.setIntakePower(0.0);
                intakeShooter.setBBServo(0.0);
                setPathState(141);
                break;
            case 141:
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
        pitch.setPitch(1);
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
        pitch.setPitch(0.87);

        follower.update();
        autonomousPathUpdate();

        turret.update();
        flywheel.update();
        intakeShooter.update(flywheel);

        telemetry.addData("Path State", pathState);
        telemetry.addData("Shoot Mode", intakeShooter.getCurrentShootMode());
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