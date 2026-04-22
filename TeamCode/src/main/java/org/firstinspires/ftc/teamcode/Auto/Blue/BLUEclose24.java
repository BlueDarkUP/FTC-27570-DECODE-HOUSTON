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

@Autonomous(name = "新路径_蓝近点多球", group = "Autonomous")
public class BLUEclose24 extends OpMode {

    private Follower follower;
    private PitchSubsystem pitch;
    private TurretSubsystem turret;
    private FlywheelSubsystem flywheel;
    private IntakeShooterSubsystem intakeShooter;

    private Timer pathTimer;
    private Timer opmodeTimer;
    private int pathState;

    // ======== 核心循环次数设置区 ========
    private final int MAX_GROUP1_LOOPS = 2; // 第一组开门嘬循环次数
    private final int MAX_GROUP2_LOOPS = 2; // 第二组开门嘬循环次数

    private int group1LoopCount = 0;
    private int group2LoopCount = 0;

    // 全局控制
    private final double TARGET_RPM = 3250.0;
    private double currentPitch = 0.6; // 统一设定的俯仰角

    // ======== 路径声明区 ========
    public PathChain fasheyuzhi;
    public PathChain xidiyipai;
    public PathChain fashediyipai;
    public PathChain xidierpai;
    public PathChain fashedierpai;
    public PathChain kaimenzuo;
    public PathChain fashekaimenzuo;
    public PathChain xidisanpai;
    public PathChain fashedisanpai;
    public PathChain fashekaimenzuoyutingkao;

    private final Pose startPose = new Pose(35.000, 135.000, Math.toRadians(180));

    public void buildPaths() {
        fasheyuzhi = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(35.000, 135.000), new Pose(45.000, 83.700)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        xidiyipai = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(45.000, 83.700), new Pose(17.000, 83.700)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        fashediyipai = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(17.000, 83.700), new Pose(46.500, 81.700), new Pose(47.900, 86.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(225))
                .build();

        xidierpai = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(47.900, 86.000), new Pose(50.000, 56.500), new Pose(13.000, 59.000)))
                .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(180))
                .build();

        fashedierpai = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(13.000, 59.000), new Pose(50.000, 57.000), new Pose(50.000, 80.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(260))
                .build();

        // 开门嘬通用路径
        kaimenzuo = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(50.000, 80.000), new Pose(45.000, 61.000), new Pose(11.500, 61.500)))
                .setLinearHeadingInterpolation(Math.toRadians(260), Math.toRadians(157))
                .build();

        // 开门嘬通用发射路径
        fashekaimenzuo = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(11.500, 61.500), new Pose(45.000, 61.000), new Pose(50.000, 80.000)))
                .setLinearHeadingInterpolation(Math.toRadians(157), Math.toRadians(260))
                .build();

        xidisanpai = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(50.000, 80.000), new Pose(51.700, 31.000), new Pose(13.000, 35.700)))
                .setLinearHeadingInterpolation(Math.toRadians(260), Math.toRadians(180))
                .build();

        fashedisanpai = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(13.000, 35.700), new Pose(60.469, 30.077), new Pose(50.000, 80.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(260))
                .build();

        // 最后一组开门嘬回去发射并停靠
        fashekaimenzuoyutingkao = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(11.500, 61.300), new Pose(45.000, 61.000), new Pose(60.000, 107.7)))
                .setLinearHeadingInterpolation(Math.toRadians(157), Math.toRadians(270))
                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            // ================= 阶段 1：发射预制球 =================
            case 10:
                turret.setTargetAngle(-59.0);
                flywheel.setTargetRPM(TARGET_RPM);
                intakeShooter.setIntakePower(0.0);
                follower.followPath(fasheyuzhi, true);
                setPathState(11);
                break;
            case 11:
                if (!follower.isBusy()) { setPathState(12); }
                break;
            case 12:
                if (pathTimer.getElapsedTimeSeconds() >= 0.1) {
                    intakeShooter.startPrecisionShoot(0.45);
                    setPathState(13);
                }
                break;
            case 13:
                if (!intakeShooter.isShootingActive()) { setPathState(20); }
                break;

            // ================= 阶段 2：吸取第一排 & 发射 =================
            case 20:
                turret.setTargetAngle(-98.0);
                intakeShooter.setBBServo(GlobalConstants.BBB_IDLE_POS);
                intakeShooter.setIntakePower(1.0);
                follower.followPath(xidiyipai, false);
                setPathState(21);
                break;
            case 21:
                if (!follower.isBusy()) { setPathState(30); }
                break;

            case 30:
                intakeShooter.setIntakePower(0.0);
                follower.followPath(fashediyipai, true);
                setPathState(31);
                break;
            case 31:
                if (!follower.isBusy()) { setPathState(32); }
                break;
            case 32:
                if (pathTimer.getElapsedTimeSeconds() >= 0.1) {
                    intakeShooter.startPrecisionShoot(0.45);
                    setPathState(33);
                }
                break;
            case 33:
                if (!intakeShooter.isShootingActive()) { setPathState(40); }
                break;

            // ================= 阶段 3：吸取第二排 & 发射 =================
            case 40:
                turret.setTargetAngle(-131.0);
                intakeShooter.setIntakePower(1.0);
                follower.followPath(xidierpai, false);
                setPathState(41);
                break;
            case 41:
                if (!follower.isBusy()) { setPathState(50); }
                break;

            case 50:
                intakeShooter.setIntakePower(0.0);
                follower.followPath(fashedierpai, true);
                setPathState(51);
                break;
            case 51:
                if (!follower.isBusy()) { setPathState(52); }
                break;
            case 52:
                if (pathTimer.getElapsedTimeSeconds() >= 0.1) {
                    intakeShooter.startPrecisionShoot(0.45);
                    setPathState(53);
                }
                break;
            case 53:
                if (!intakeShooter.isShootingActive()) {
                    group1LoopCount = 0;
                    setPathState(60);
                }
                break;

            // ================= 阶段 4：开门嘬（第一组，可循环） =================
            case 60:
                turret.setTargetAngle(-132.0);
                intakeShooter.setIntakePower(1.0); // 开启吸球
                follower.followPath(kaimenzuo, false);
                setPathState(61);
                break;
            case 61:
                // 到达吸球点后，进入 62 状态开始倒计时
                if (!follower.isBusy()) { setPathState(62); }
                break;
            case 62:
                // ★★★ 原地等待 1.5 秒，让 Intake 充分吸球 ★★★
                if (pathTimer.getElapsedTimeSeconds() >= 1.5) {
                    setPathState(70);
                }
                break;

            case 70:
                intakeShooter.setIntakePower(0.0);
                follower.followPath(fashekaimenzuo, true);
                setPathState(71);
                break;
            case 71:
                if (!follower.isBusy()) { setPathState(72); }
                break;
            case 72:
                if (pathTimer.getElapsedTimeSeconds() >= 0.1) {
                    intakeShooter.startPrecisionShoot(0.45);
                    setPathState(73);
                }
                break;
            case 73:
                if (!intakeShooter.isShootingActive()) {
                    group1LoopCount++;
                    if (group1LoopCount < MAX_GROUP1_LOOPS) {
                        setPathState(60);
                    } else {
                        setPathState(80);
                    }
                }
                break;

            // ================= 阶段 5：吸取第三排 & 发射 =================
            case 80:
                turret.setTargetAngle(-132.0);
                intakeShooter.setIntakePower(1.0);
                follower.followPath(xidisanpai, false);
                setPathState(81);
                break;
            case 81:
                if (!follower.isBusy()) { setPathState(90); }
                break;

            case 90:
                intakeShooter.setIntakePower(0.0);
                follower.followPath(fashedisanpai, true);
                setPathState(91);
                break;
            case 91:
                if (!follower.isBusy()) { setPathState(92); }
                break;
            case 92:
                if (pathTimer.getElapsedTimeSeconds() >= 0.1) {
                    intakeShooter.startPrecisionShoot(0.45);
                    setPathState(93);
                }
                break;
            case 93:
                if (!intakeShooter.isShootingActive()) {
                    group2LoopCount = 0;
                    setPathState(100);
                }
                break;

            // ================= 阶段 6：开门嘬（第二组，最后含停靠） =================
            case 100:
                turret.setTargetAngle(-132.0);
                intakeShooter.setIntakePower(1.0); // 开启吸球
                follower.followPath(kaimenzuo, false);
                setPathState(101);
                break;
            case 101:
                // 到达吸球点后，进入 102 状态开始倒计时
                if (!follower.isBusy()) { setPathState(102); }
                break;
            case 102:
                // ★★★ 原地等待 1.5 秒，让 Intake 充分吸球 ★★★
                if (pathTimer.getElapsedTimeSeconds() >= 1.5) {
                    setPathState(110);
                }
                break;

            case 110:
                intakeShooter.setIntakePower(0.0);
                if (group2LoopCount == MAX_GROUP2_LOOPS - 1) {
                    turret.setTargetAngle(-122);
                    follower.followPath(fashekaimenzuoyutingkao, true);
                } else {
                    turret.setTargetAngle(-132.0);
                    follower.followPath(fashekaimenzuo, true);
                }
                setPathState(111);
                break;
            case 111:
                if (!follower.isBusy()) { setPathState(112); }
                break;
            case 112:
                if (pathTimer.getElapsedTimeSeconds() >= 0.1) {
                    intakeShooter.startPrecisionShoot(0.45);
                    setPathState(113);
                }
                break;
            case 113:
                if (!intakeShooter.isShootingActive()) {
                    group2LoopCount++;
                    if (group2LoopCount < MAX_GROUP2_LOOPS) {
                        setPathState(100);
                    } else {
                        setPathState(120);
                    }
                }
                break;

            // ================= 结束阶段 =================
            case 120:
                flywheel.setTargetRPM(0.0);
                intakeShooter.setIntakePower(0.0);
                turret.setTargetAngle(0);
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
        pitch.setPitch(currentPitch);
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
        pitch.setPitch(currentPitch);

        follower.update();
        autonomousPathUpdate();

        turret.update();
        flywheel.update();
        intakeShooter.update(flywheel);

        telemetry.addData("Path State", pathState);
        telemetry.addData("RPM Current/Target", "%.1f / %.1f", flywheel.getCurrentRPM(), flywheel.getTargetRPM());
        telemetry.addData("G1 Loops (开门嘬1)", "%d / %d", group1LoopCount, MAX_GROUP1_LOOPS);
        telemetry.addData("G2 Loops (开门嘬2)", "%d / %d", group2LoopCount, MAX_GROUP2_LOOPS);
        telemetry.update();
    }

    @Override
    public void stop() {
        turret.stop();
        flywheel.stop();
        intakeShooter.stop();
    }
}