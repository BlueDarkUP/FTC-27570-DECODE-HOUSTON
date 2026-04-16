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

@Autonomous(name = "蓝方远点吸第三排", group = "Autonomous")
public class BLUEfarLONG extends OpMode {

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

    // ================= 路径声明 (已修复变量名重复问题) =================
    public PathChain xidisanpai;
    public PathChain huiqufashe;

    // 第一组角落
    public PathChain zhunbeixijiaoluo;
    public PathChain xijiaoluo1;
    public PathChain huilaifashe1;

    // 第二组角落
    public PathChain xijiaoluo2;
    public PathChain jixuxijiaoluo1;
    public PathChain huilaifashe2;

    // 第三组角落
    public PathChain xijiaoluo3;
    public PathChain jixuxijiaoluo2;
    public PathChain huilaifashe3;

    // 第四组角落
    public PathChain xijiaoluo4;
    public PathChain jixuxijiaoluo3;
    public PathChain huilaifashe4;

    // 停靠
    public PathChain tingkao;

    // 起始点 (根据你的 xidisanpai 第一条曲线起点设定)
    private final Pose startPose = new Pose(57.077, 8.769, Math.toRadians(180));

    public void buildPaths() {
        xidisanpai = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(57.077, 8.769),
                                new Pose(62.000, 39.000),
                                new Pose(16.769, 35.846)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        huiqufashe = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(16.769, 35.846),
                                new Pose(60.000, 14.800)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        zhunbeixijiaoluo = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(60.000, 14.800),
                                new Pose(9.500, 26.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                .build();

        xijiaoluo1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(9.500, 26.000),
                                new Pose(11.000, 13.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        huilaifashe1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(11.000, 13.000),
                                new Pose(31.000, 22.000),
                                new Pose(60.000, 14.800)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(180))
                .build();

        xijiaoluo2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(60.000, 14.800),
                                new Pose(10.000, 12.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        jixuxijiaoluo1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(10.000, 12.000),
                                new Pose(22.000, 15.462),
                                new Pose(10.000, 20.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        huilaifashe2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(10.000, 20.000),
                                new Pose(60.000, 14.800)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        xijiaoluo3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(60.000, 14.800),
                                new Pose(10.000, 14.800)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        jixuxijiaoluo2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(10.000, 14.800),
                                new Pose(22.000, 17.000),
                                new Pose(10.000, 24.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        huilaifashe3 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(10.000, 24.000),
                                new Pose(60.000, 14.800)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        xijiaoluo4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(60.000, 14.800),
                                new Pose(10.000, 12.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        jixuxijiaoluo3 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(10.000, 12.000),
                                new Pose(22.000, 15.000),
                                new Pose(10.000, 14.800)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        huilaifashe4 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(10.000, 14.800),
                                new Pose(60.000, 14.800)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        tingkao = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(60.000, 14.800),
                                new Pose(38.600, 33.400)
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
            // ================== 1. 开局云台-77并原地发射1秒 ==================
            case 10:
                turret.setTargetAngle(-69.0);
                // 俯仰角和RPM已在loop()中全局锁定
                setPathState(11);
                break;
            case 11:
                // 等待0.5秒让云台和摩擦轮达到预设值
                if (pathTimer.getElapsedTimeSeconds() >= 2.5) {
                    intakeShooter.startPrecisionShoot(0.8);
                    setPathState(12);
                }
                break;
            case 12:
                // 保证发射至少持续 1 秒
                if (!intakeShooter.isShootingActive() && pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    setPathState(20);
                }
                break;

            // ================== 2. 吸第三排 & 回去发射 ==================
            case 20:
                follower.followPath(xidisanpai, false);
                intakeShooter.setIntakePower(1.0); // 开启吸取
                setPathState(21);
                break;
            case 21:
                if (!follower.isBusy()) {
                    setPathState(30);
                }
                break;
            case 30:
                follower.followPath(huiqufashe, true);
                intakeShooter.setIntakePower(0.0);
                setPathState(31);
                break;
            case 31:
                if (!follower.isBusy()) {
                    setPathState(32);
                }
                break;
            case 32:
                // 等待 0.1 秒底盘微调后发射
                if (pathTimer.getElapsedTimeSeconds() >= 0.1) {
                    intakeShooter.startPrecisionShoot(0.8);
                    setPathState(33);
                }
                break;
            case 33:
                if (!intakeShooter.isShootingActive()) {
                    setPathState(40);
                }
                break;

            // ================== 3. 准备吸角落 & 吸角落1 & 回来发射1 ==================
            case 40:
                follower.followPath(zhunbeixijiaoluo, false);
                intakeShooter.setIntakePower(1.0); // 开启吸取
                setPathState(41);
                break;
            case 41:
                if (!follower.isBusy()) {
                    follower.followPath(xijiaoluo1, false);
                    setPathState(42);
                }
                break;
            case 42:
                if (!follower.isBusy()) {
                    setPathState(50);
                }
                break;
            case 50:
                follower.followPath(huilaifashe1, true);
                intakeShooter.setIntakePower(0.0);
                setPathState(51);
                break;
            case 51:
                if (!follower.isBusy()) {
                    setPathState(52);
                }
                break;
            case 52:
                // 等待 0.1 秒底盘微调后发射
                if (pathTimer.getElapsedTimeSeconds() >= 0.1) {
                    intakeShooter.startPrecisionShoot(0.8);
                    setPathState(53);
                }
                break;
            case 53:
                if (!intakeShooter.isShootingActive()) {
                    setPathState(60);
                }
                break;

            // ================== 4. 吸角落2 & 继续吸角落1 & 回来发射2 ==================
            case 60:
                follower.followPath(xijiaoluo2, false);
                intakeShooter.setIntakePower(1.0); // 开启吸取
                setPathState(61);
                break;
            case 61:
                if (!follower.isBusy()) {
                    follower.followPath(jixuxijiaoluo1, false);
                    setPathState(62);
                }
                break;
            case 62:
                if (!follower.isBusy()) {
                    setPathState(70);
                }
                break;
            case 70:
                follower.followPath(huilaifashe2, true);
                intakeShooter.setIntakePower(0.0);
                setPathState(71);
                break;
            case 71:
                if (!follower.isBusy()) {
                    setPathState(72);
                }
                break;
            case 72:
                // 等待 0.1 秒底盘微调后发射
                if (pathTimer.getElapsedTimeSeconds() >= 0.1) {
                    intakeShooter.startPrecisionShoot(0.8);
                    setPathState(73);
                }
                break;
            case 73:
                if (!intakeShooter.isShootingActive()) {
                    setPathState(80);
                }
                break;

            // ================== 5. 吸角落3 & 继续吸角落2 & 回来发射3 ==================
            case 80:
                follower.followPath(xijiaoluo3, false);
                intakeShooter.setIntakePower(1.0); // 开启吸取
                setPathState(81);
                break;
            case 81:
                if (!follower.isBusy()) {
                    follower.followPath(jixuxijiaoluo2, false);
                    setPathState(82);
                }
                break;
            case 82:
                if (!follower.isBusy()) {
                    setPathState(90);
                }
                break;
            case 90:
                follower.followPath(huilaifashe3, true);
                intakeShooter.setIntakePower(0.0);
                setPathState(91);
                break;
            case 91:
                if (!follower.isBusy()) {
                    setPathState(92);
                }
                break;
            case 92:
                // 等待 0.1 秒底盘微调后发射
                if (pathTimer.getElapsedTimeSeconds() >= 0.1) {
                    intakeShooter.startPrecisionShoot(0.8);
                    setPathState(93);
                }
                break;
            case 93:
                if (!intakeShooter.isShootingActive()) {
                    setPathState(100);
                }
                break;

            // ================== 6. 吸角落4 & 继续吸角落3 & 回来发射4 ==================
            case 100:
                follower.followPath(xijiaoluo4, false);
                intakeShooter.setIntakePower(1.0); // 开启吸取
                setPathState(101);
                break;
            case 101:
                if (!follower.isBusy()) {
                    follower.followPath(jixuxijiaoluo3, false);
                    setPathState(102);
                }
                break;
            case 102:
                if (!follower.isBusy()) {
                    setPathState(110);
                }
                break;
            case 110:
                follower.followPath(huilaifashe4, true);
                intakeShooter.setIntakePower(0.0);
                setPathState(111);
                break;
            case 111:
                if (!follower.isBusy()) {
                    setPathState(112);
                }
                break;
            case 112:
                // 等待 0.1 秒底盘微调后发射
                if (pathTimer.getElapsedTimeSeconds() >= 0.1) {
                    intakeShooter.startPrecisionShoot(0.8);
                    setPathState(113);
                }
                break;
            case 113:
                if (!intakeShooter.isShootingActive()) {
                    setPathState(120);
                }
                break;

            // ================== 7. 停靠 ==================
            case 120:
                follower.followPath(tingkao, true);
                turret.setTargetAngle(0.0); // 停靠时回正云台
                setPathState(121);
                break;
            case 121:
                if (!follower.isBusy()) {
                    setPathState(130);
                }
                break;

            // ================== 结束 ==================
            case 130:
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
        // Init 阶段即拉起参数
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
        // ====== 全程锁死的硬件参数 ======
        pitch.setPitch(1.0);
        flywheel.setTargetRPM(4700.0);

        follower.update();
        autonomousPathUpdate();

        turret.update();
        flywheel.update();
        intakeShooter.update(flywheel);

        telemetry.addData("Path State", pathState);
        telemetry.addData("Turret Angle", turret.getTargetAngle());
        telemetry.addData("Shoot Mode", intakeShooter.getCurrentShootMode());
        telemetry.update();
    }

    @Override
    public void stop() {
        turret.stop();
        flywheel.stop();
        intakeShooter.stop();
    }
}