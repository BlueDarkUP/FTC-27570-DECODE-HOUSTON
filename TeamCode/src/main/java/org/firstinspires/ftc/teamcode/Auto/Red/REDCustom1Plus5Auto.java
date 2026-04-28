package org.firstinspires.ftc.teamcode.Auto.Red;

import com.pedropathing.follower.Follower;
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

@Autonomous(name = "红方远点1+5自动", group = "Autonomous")
public class REDCustom1Plus5Auto extends OpMode {

    private Follower follower;
    private PitchSubsystem pitch;
    private TurretSubsystem turret;
    private FlywheelSubsystem flywheel;
    private IntakeShooterSubsystem intakeShooter;
    private DcMotorEx rawTurretMotor;

    private Timer pathTimer;
    private Timer opmodeTimer;
    private int pathState;

    // ================= 自定义配置区域 =================
    private final int TOTAL_CYCLES = 5; // A/B交替循环次数
    private int cycleCount = 0;

    // 核心参数设定
    private final double TARGET_RPM = 4090.0;
    private final double TARGET_PITCH = 1.0;
    private final double TURRET_SHOOT_ANGLE = 69.0; // 蓝方 -68.0 取反
    private final double CHASSIS_SETTLE_TIME = 0.3; // 底盘到位校准时间
    private final double INTAKE_WAIT_TIME = 0.3;    // 角落吸取等待时间
    // ==================================================

    public PathChain startToShoot;

    // 第三排路径
    public PathChain shootToPreIntake3; // (89,12) -> (104,34)
    public PathChain preIntake3ToCorner3; // (104,34) -> (134,34)
    public PathChain corner3ToShoot;      // (134,34) -> (89,12)

    // A/B角落路径
    public PathChain shootToCornerA, cornerAToShoot;
    public PathChain shootToCornerB, cornerBToShoot;

    public PathChain shootToPark;

    // 坐标X轴全部执行 144 - X，朝向180度镜像为0度
    private final Pose startPose = new Pose(88.000, 8.000, Math.toRadians(0));
    private final Pose shootPose = new Pose(89.000, 12.000, Math.toRadians(0));
    private final Pose preIntake3Pose = new Pose(104.000, 34.000, Math.toRadians(0));
    private final Pose corner3Pose = new Pose(134.000, 34.000, Math.toRadians(0));
    private final Pose cornerAPose = new Pose(134.000, 10.000, Math.toRadians(0));
    private final Pose cornerBPose = new Pose(132.000, 23.000, Math.toRadians(0));

    public void buildPaths() {
        // 起点到发射点
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        // 第三排路径
        shootToPreIntake3 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, preIntake3Pose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        preIntake3ToCorner3 = follower.pathBuilder()
                .addPath(new BezierLine(preIntake3Pose, corner3Pose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        corner3ToShoot = follower.pathBuilder()
                .addPath(new BezierLine(corner3Pose, shootPose))
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        // A/B 循环路径
        shootToCornerA = follower.pathBuilder().addPath(new BezierLine(shootPose, cornerAPose)).setConstantHeadingInterpolation(Math.toRadians(0)).build();
        cornerAToShoot = follower.pathBuilder().addPath(new BezierLine(cornerAPose, shootPose)).setConstantHeadingInterpolation(Math.toRadians(0)).build();
        shootToCornerB = follower.pathBuilder().addPath(new BezierLine(shootPose, cornerBPose)).setConstantHeadingInterpolation(Math.toRadians(0)).build();
        cornerBToShoot = follower.pathBuilder().addPath(new BezierLine(cornerBPose, shootPose)).setConstantHeadingInterpolation(Math.toRadians(0)).build();

        // 停靠
        shootToPark = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, new Pose(85.000, 35.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            // ================== 阶段1：初始预制发射 ==================
            case 10:
                flywheel.setTargetRPM(TARGET_RPM);
                turret.setTargetAngle(TURRET_SHOOT_ANGLE);
                follower.followPath(startToShoot, true);
                setPathState(11);
                break;
            case 11:
                if (!follower.isBusy()) setPathState(12);
                break;
            case 12:
                if (pathTimer.getElapsedTimeSeconds() >= CHASSIS_SETTLE_TIME) setPathState(13);
                break;
            case 13:
                if (flywheel.getCurrentRPM() >= 4070.0) {
                    intakeShooter.startPrecisionShoot(GlobalConstants.SHOOT_TIME_NORMAL);
                    setPathState(14);
                }
                break;
            case 14:
                if (!intakeShooter.isShootingActive()) {
                    // 预制发射完毕，直接跳转去吸第三排
                    setPathState(30);
                }
                break;

            // ================== 阶段2：第三排特殊吸取 ==================
            case 30:
                // 从发射点前往准备点 (104, 34)
                follower.followPath(shootToPreIntake3, true);
                setPathState(31);
                break;
            case 31:
                if (!follower.isBusy()) {
                    follower.setMaxPower(0.9);
                    // 到达准备点末尾，立即打开 Intake 并前往吸取点 (134, 34)
                    intakeShooter.setIntakePower(1.0);
                    follower.followPath(preIntake3ToCorner3, false);
                    setPathState(32);
                }
                break;
            case 32:
                if (!follower.isBusy()) {
                    // pathTimer 已被 setPathState 重置
                    setPathState(33);
                }
                break;
            case 33:
                // 吸取等待
                if (pathTimer.getElapsedTimeSeconds() >= INTAKE_WAIT_TIME) {
                    follower.followPath(corner3ToShoot, true);
                    setPathState(34);
                }
                break;
            case 34:
                if (!follower.isBusy()) {
                    intakeShooter.setIntakePower(0.0);
                    setPathState(35);
                }
                break;
            case 35:
                // 发射前底盘校准
                if (pathTimer.getElapsedTimeSeconds() >= CHASSIS_SETTLE_TIME) {
                    intakeShooter.startPrecisionShoot(GlobalConstants.SHOOT_TIME_NORMAL);
                    setPathState(36);
                }
                break;
            case 36:
                if (!intakeShooter.isShootingActive()) {
                    // 第三排打完，初始化循环计数器，进入 A/B 循环
                    cycleCount = 0;
                    setPathState(20);
                }
                break;

            // ================== 阶段3：5次 AB 交替循环 ==================
            case 20:
                if (cycleCount >= TOTAL_CYCLES) {
                    setPathState(40); // 循环结束，去停靠
                } else {
                    if (cycleCount % 2 == 0) follower.followPath(shootToCornerA, false);
                    else follower.followPath(shootToCornerB, false);
                    intakeShooter.setIntakePower(1.0);
                    setPathState(21);
                }
                break;
            case 21:
                if (!follower.isBusy()) setPathState(22);
                break;
            case 22:
                if (pathTimer.getElapsedTimeSeconds() >= INTAKE_WAIT_TIME) setPathState(23);
                break;
            case 23:
                if (cycleCount % 2 == 0) follower.followPath(cornerAToShoot, true);
                else follower.followPath(cornerBToShoot, true);
                setPathState(24);
                break;
            case 24:
                if (!follower.isBusy()) {
                    intakeShooter.setIntakePower(0.0);
                    setPathState(25);
                }
                break;
            case 25:
                if (pathTimer.getElapsedTimeSeconds() >= CHASSIS_SETTLE_TIME) {
                    intakeShooter.startPrecisionShoot(GlobalConstants.SHOOT_TIME_NORMAL);
                    setPathState(26);
                }
                break;
            case 26:
                if (!intakeShooter.isShootingActive()) {
                    cycleCount++;
                    setPathState(20); // 继续下一个循环
                }
                break;

            // ================== 阶段4：停靠复位 ==================
            case 40:
                follower.followPath(shootToPark, true);
                turret.setTargetAngle(0.0);
                setPathState(41);
                break;
            case 41:
                if (!follower.isBusy()) setPathState(42);
                break;
            case 42:
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
        cycleCount = 0;

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
        pitch.setPitch(TARGET_PITCH);
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
        pitch.setPitch(TARGET_PITCH);
        follower.update();
        autonomousPathUpdate();
        turret.update();
        flywheel.update();
        intakeShooter.update(flywheel);

        telemetry.addData("Path State", pathState);
        telemetry.addData("Cycles", "%d / %d", cycleCount, TOTAL_CYCLES);
        telemetry.addData("RPM", "%.1f / %.1f", flywheel.getCurrentRPM(), flywheel.getTargetRPM());
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