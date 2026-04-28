package org.firstinspires.ftc.teamcode.Auto.Blue;

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

@Autonomous(name = "蓝方远点自动", group = "Autonomous")
public class BLUE7far extends OpMode {

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
    // 你可以在这里自由设置循环的总次数（AB交替的总和）
    // 例如：7次即为 A-B-A-B-A-B-A
    private final int TOTAL_CYCLES = 6;
    private int cycleCount = 0; // 当前已执行的循环次数记录

    // 核心参数设定
    private final double TARGET_RPM = 4090.0;
    private final double TARGET_PITCH = 1;
    private final double TURRET_SHOOT_ANGLE = -68.0;
    private final double CHASSIS_SETTLE_TIME = 0.3; // 底盘到位后的校准等待时间 (秒)
    // ==================================================

    // 拆分后的独立路径
    public PathChain startToShoot;
    public PathChain shootToCornerA;
    public PathChain cornerAToShoot;
    public PathChain shootToCornerB;
    public PathChain cornerBToShoot;
    public PathChain shootToPark;

    private final Pose startPose = new Pose(56.000, 8.000, Math.toRadians(180));
    private final Pose shootPose = new Pose(55.000, 12.000, Math.toRadians(180));
    private final Pose cornerAPose = new Pose(10.000, 10.000, Math.toRadians(180));
    private final Pose cornerBPose = new Pose(12.000, 23.000, Math.toRadians(180));

    public void buildPaths() {
        // 1. 起点 -> 发射点 (用于发预制)
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        // 2. 发射点 -> A角落
        shootToCornerA = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, cornerAPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // 3. A角落 -> 发射点
        cornerAToShoot = follower.pathBuilder()
                .addPath(new BezierLine(cornerAPose, shootPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // 4. 发射点 -> B角落
        shootToCornerB = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, cornerBPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // 5. B角落 -> 发射点
        cornerBToShoot = follower.pathBuilder()
                .addPath(new BezierLine(cornerBPose, shootPose))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        // 6. 发射点 -> 停靠
        shootToPark = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, new Pose(59.000, 35.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))
                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            // ================== 初始预制发射阶段 ==================
            case 10:
                // 开启飞轮至4100，云台锁定-71
                flywheel.setTargetRPM(TARGET_RPM);
                turret.setTargetAngle(TURRET_SHOOT_ANGLE);
                intakeShooter.setIntakePower(0.0);

                // true表示holdEnd，停稳再发
                follower.followPath(startToShoot, true);
                setPathState(11);
                break;
            case 11:
                // 等待底盘到达发射点
                if (!follower.isBusy()) {
                    setPathState(12); // 进入校准等待
                }
                break;
            case 12:
                // 底盘到位后，硬性等待 0.3 秒让底盘校准稳定
                if (pathTimer.getElapsedTimeSeconds() >= CHASSIS_SETTLE_TIME) {
                    setPathState(13);
                }
                break;
            case 13:
                // 等待飞轮加速到至少4000转（仅第一次发射需要严格检测此条件）
                if (flywheel.getCurrentRPM() >= 4070.0) {
                    intakeShooter.startPrecisionShoot(GlobalConstants.SHOOT_TIME_NORMAL);
                    setPathState(14);
                }
                break;
            case 14:
                // 等待发射动作结束
                if (!intakeShooter.isShootingActive()) {
                    cycleCount = 0; // 重置循环计数器
                    setPathState(20);
                }
                break;

            // ================== 核心循环阶段 ==================
            case 20:
                // 检查是否已经完成了所有的吸取发射次数
                if (cycleCount >= TOTAL_CYCLES) {
                    setPathState(40); // 去停靠
                } else {
                    // 判断去A点还是B点 (偶数去A，奇数去B)
                    if (cycleCount % 2 == 0) {
                        follower.followPath(shootToCornerA, false); // false代表不holdEnd
                    } else {
                        follower.followPath(shootToCornerB, false); // false代表不holdEnd
                    }

                    intakeShooter.setIntakePower(1.0); // 开启吸入
                    setPathState(21);
                }
                break;
            case 21:
                // 到达角落点 (不要求完全停稳，路径走完即可)
                if (!follower.isBusy()) {
                    // setPathState 会自动重置 pathTimer
                    setPathState(22);
                }
                break;
            case 22:
                // 在角落不HoldEnd，但硬性等待1秒让球进入球路
                if (pathTimer.getElapsedTimeSeconds() >= 0.3) {
                    setPathState(23);
                }
                break;
            case 23:
                // 返回发射点 (根据刚才去的哪个点决定返回路径)
                if (cycleCount % 2 == 0) {
                    follower.followPath(cornerAToShoot, true); // 发射点需要holdEnd
                } else {
                    follower.followPath(cornerBToShoot, true); // 发射点需要holdEnd
                }
                setPathState(24);
                break;
            case 24:
                // 等待到达停靠发射点
                if (!follower.isBusy()) {
                    intakeShooter.setIntakePower(0.0); // 关吸
                    setPathState(25); // 进入底盘校准等待
                }
                break;
            case 25:
                // 底盘到位后，硬性等待 0.3 秒让底盘校准稳定
                if (pathTimer.getElapsedTimeSeconds() >= CHASSIS_SETTLE_TIME) {
                    intakeShooter.startPrecisionShoot(GlobalConstants.SHOOT_TIME_NORMAL);
                    setPathState(26);
                }
                break;
            case 26:
                // 确保射击完成，增加一次计数，然后跳回State 20继续循环
                if (!intakeShooter.isShootingActive()) {
                    cycleCount++;
                    setPathState(20);
                }
                break;

            // ================== 结束与停靠阶段 ==================
            case 40:
                follower.followPath(shootToPark, true);
                turret.setTargetAngle(0.0); // 云台复位
                setPathState(41);
                break;
            case 41:
                if (!follower.isBusy()) {
                    setPathState(42);
                }
                break;
            case 42:
                // 全车复位
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
        // 初始化时俯仰可根据需要设置，这里直接用要求的目标值或默认值
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
        // 全程强制锁定俯仰角度为 1
        pitch.setPitch(TARGET_PITCH);

        follower.update();
        autonomousPathUpdate();

        turret.update();
        flywheel.update();
        intakeShooter.update(flywheel);

        telemetry.addData("Path State", pathState);
        telemetry.addData("Cycles Completed", "%d / %d", cycleCount, TOTAL_CYCLES);
        telemetry.addData("Current Target", (cycleCount >= TOTAL_CYCLES) ? "Park" : ((cycleCount % 2 == 0) ? "Corner A" : "Corner B"));
        telemetry.addData("RPM Current/Target", "%.1f / %.1f", flywheel.getCurrentRPM(), flywheel.getTargetRPM());
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