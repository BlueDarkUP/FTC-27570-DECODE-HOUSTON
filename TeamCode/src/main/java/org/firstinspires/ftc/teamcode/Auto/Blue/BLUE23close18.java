package org.firstinspires.ftc.teamcode.Auto.Blue;

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

@Autonomous(name = "蓝方近点18球自动", group = "Autonomous")
public class BLUE23close18 extends OpMode {

    private Follower follower;
    private PitchSubsystem pitch;
    private TurretSubsystem turret;
    private FlywheelSubsystem flywheel;
    private IntakeShooterSubsystem intakeShooter;
    private DcMotorEx rawTurretMotor;

    private Timer pathTimer;
    private Timer opmodeTimer;
    private int pathState;

    private final int MAX_DOOR_EXTRACT_LOOPS = 2;
    private int doorExtractLoopCount = 0;

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
    public PathChain Returntofixedlaunchpoint;

    private final Pose startPose = new Pose(35.000, 134.600, Math.toRadians(180));

    public void buildPaths() {
        diyigepaoda = follower.pathBuilder()
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

        fashedierpai = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(11.000, 59.000), new Pose(45.000, 60.000), new Pose(60.000, 83.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(226))
                .build();

        kaimenzuo = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(60.000, 83.000),
                                new Pose(45.000, 61.3000),
                                new Pose(11.5, 60)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(157))
                .build();

        fashekaimenzuo = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(11.5, 60), new Pose(35.000, 47.000), new Pose(60.000, 83.000)))
                .setLinearHeadingInterpolation(Math.toRadians(157), Math.toRadians(226))
                .build();

        zhunbeixidiyipai = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60.000, 83.000), new Pose(45.000, 83.000)))
                .setLinearHeadingInterpolation(Math.toRadians(226), Math.toRadians(180))
                .setNoDeceleration()
                .build();

        xidiyipai = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(45.000, 83.000), new Pose(19.000, 83.000)))
                .setTangentHeadingInterpolation()
                .build();

        fashediyipai = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(19.000, 83.000), new Pose(60.000, 83.000)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(230))
                .build();

        zhunbeixidisanpai = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(60.000, 83.000), new Pose(50.000, 36.000)))
                .setLinearHeadingInterpolation(Math.toRadians(230), Math.toRadians(180))
                .setNoDeceleration()
                .build();

        xidisanpai = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(50.000, 36.000), new Pose(12.000, 36.000)))
                .setTangentHeadingInterpolation()
                .build();

        Returntofixedlaunchpoint = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(13.000, 35.800), new Pose(59.923, 107.923)))
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
                flywheel.setTargetRPM(GlobalConstants.AUTO_RPM_NORMAL);
                follower.followPath(diyigepaoda, true);
                turret.setTargetAngle(-47.0);
                intakeShooter.setIntakePower(0.0);
                setPathState(11);
                break;
            case 11:
                if (!follower.isBusy()) {
                    setPathState(115);
                }
                break;
            case 115:
                if (pathTimer.getElapsedTimeSeconds() >= 0) {
                    intakeShooter.startPrecisionShoot(GlobalConstants.SHOOT_TIME_NORMAL);
                    setPathState(12);
                }
                break;
            case 12:
                if (!intakeShooter.isShootingActive()) {
                    follower.setMaxPower(1.0);
                    setPathState(20);
                }
                break;

            case 20:
                follower.setMaxPower(1);
                follower.followPath(xidierpai, false);
                intakeShooter.setBBServo(GlobalConstants.BBB_IDLE_POS);
                intakeShooter.setIntakePower(1.0);
                flywheel.setTargetRPM(GlobalConstants.AUTO_RPM_NORMAL);
                setPathState(21);
                break;
            case 21:
                if (!follower.isBusy()) { setPathState(30); }
                break;

            case 30:
                follower.followPath(fashedierpai, true);
                turret.setTargetAngle(-92.0);
                flywheel.setTargetRPM(GlobalConstants.AUTO_RPM_NORMAL);
                intakeShooter.setIntakePower(0.0);
                setPathState(31);
                break;
            case 31:
                if (!follower.isBusy()) {
                    setPathState(315);
                }
                break;
            case 315:
                if (pathTimer.getElapsedTimeSeconds() >= 0.2) {
                    intakeShooter.startPrecisionShoot(GlobalConstants.SHOOT_TIME_NORMAL);
                    setPathState(32);
                }
                break;
            case 32:
                if (!intakeShooter.isShootingActive()) {
                    doorExtractLoopCount = 0;
                    setPathState(40);
                }
                break;

            case 40:
                follower.followPath(kaimenzuo, true);
                intakeShooter.setIntakePower(1.0);
                flywheel.setTargetRPM(GlobalConstants.AUTO_RPM_NORMAL);
                setPathState(41);
                break;
            case 41:
                if (!follower.isBusy()) {
                    setPathState(42);
                }
                break;
            case 42:
                if (pathTimer.getElapsedTimeSeconds() >= 1.5) {
                    setPathState(50);
                }
                break;

            case 50:
                follower.followPath(fashekaimenzuo, true);
                turret.setTargetAngle(-92.0);
                flywheel.setTargetRPM(GlobalConstants.AUTO_RPM_NORMAL);
                intakeShooter.setIntakePower(1.0);
                setPathState(51);
                break;
            case 51:
                if (!follower.isBusy()) {
                    setPathState(515);
                }
                break;
            case 515:
                if (pathTimer.getElapsedTimeSeconds() >= 0.4) {
                    intakeShooter.startPrecisionShoot(GlobalConstants.SHOOT_TIME_NORMAL);
                    setPathState(52);
                }
                break;
            case 52:
                if (!intakeShooter.isShootingActive()) {
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
                flywheel.setTargetRPM(GlobalConstants.AUTO_RPM_NORMAL);
                intakeShooter.setIntakePower(1.0);
                setPathState(61);
                break;
            case 61:
                if (!follower.isBusy() || follower.atParametricEnd()) {
                    setPathState(70);
                }
                break;

            case 70:
                follower.followPath(xidiyipai, false);
                intakeShooter.setIntakePower(1.0);
                setPathState(71);
                break;
            case 71:
                if (!follower.isBusy()) { setPathState(80); }
                break;

            case 80:
                follower.followPath(fashediyipai, true);
                turret.setTargetAngle(-92.0);
                flywheel.setTargetRPM(GlobalConstants.AUTO_RPM_NORMAL);
                intakeShooter.setIntakePower(0.0);
                setPathState(81);
                break;
            case 81:
                if (!follower.isBusy()) {
                    setPathState(815);
                }
                break;
            case 815:
                if (pathTimer.getElapsedTimeSeconds() >= 0.2) {
                    intakeShooter.startPrecisionShoot(GlobalConstants.SHOOT_TIME_NORMAL);
                    setPathState(82);
                }
                break;
            case 82:
                if (!intakeShooter.isShootingActive()) { setPathState(90); }
                break;

            case 90:
                follower.followPath(zhunbeixidisanpai, false);
                flywheel.setTargetRPM(GlobalConstants.AUTO_RPM_NORMAL);
                intakeShooter.setIntakePower(1.0);
                setPathState(91);
                break;
            case 91:
                if (!follower.isBusy() || follower.atParametricEnd()) {
                    setPathState(100);
                }
                break;

            case 100:
                follower.followPath(xidisanpai, false);
                intakeShooter.setIntakePower(1.0);
                setPathState(101);
                break;
            case 101:
                if (!follower.isBusy()) { setPathState(110); }
                break;

            case 110:
                follower.followPath(Returntofixedlaunchpoint, true);
                turret.setTargetAngle(-25.0);
                flywheel.setTargetRPM(3100);
                intakeShooter.setIntakePower(0.0);
                setPathState(111);
                break;
            case 111:
                if (!follower.isBusy()) {
                    setPathState(1115);
                }
                break;
            case 1115:
                if (pathTimer.getElapsedTimeSeconds() >= 0.2) {
                    intakeShooter.startPrecisionShoot(GlobalConstants.SHOOT_TIME_NORMAL);
                    setPathState(112);
                }
                break;
            case 112:
                if (!intakeShooter.isShootingActive()) { setPathState(120); }
                break;

            case 120:
                turret.setTargetAngle(0.0);
                flywheel.setTargetRPM(0.0);
                intakeShooter.setIntakePower(0.0);
                intakeShooter.setBBServo(GlobalConstants.BBB_IDLE_POS);
                setPathState(121);
                break;
            case 121:
                setPathState(-1);
                break;
        }
    }

    @Override
    public void init() {
        RobotStateStorage.clear();
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        doorExtractLoopCount = 0;
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
        pitch.setPitch(GlobalConstants.PITCH_POS_INTAKE_DEEP);
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
        pitch.setPitch(GlobalConstants.PITCH_POS_TRANSIT);
        follower.update();
        autonomousPathUpdate();
        turret.update();
        flywheel.update();
        intakeShooter.update(flywheel);
        telemetry.addData("Path State", pathState);
        telemetry.addData("Shoot Mode", intakeShooter.getCurrentShootMode());
        telemetry.addData("RPM Current/Target", "%.1f / %.1f", flywheel.getCurrentRPM(), flywheel.getTargetRPM());
        telemetry.addData("Door Loop Progress", "%d / %d", doorExtractLoopCount, MAX_DOOR_EXTRACT_LOOPS);
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