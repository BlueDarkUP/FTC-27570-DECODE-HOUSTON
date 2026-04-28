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

@Autonomous(name = "蓝方开一次门", group = "Autonomous")
public class BLUEclose1gate extends OpMode {

    private Follower follower;
    private PitchSubsystem pitch;
    private TurretSubsystem turret;
    private FlywheelSubsystem flywheel;
    private IntakeShooterSubsystem intakeShooter;
    private DcMotorEx rawTurretMotor;

    private Timer pathTimer;
    private Timer opmodeTimer;
    private int pathState;

    public PathChain fasheyuzhi;
    public PathChain xidiyipai;
    public PathChain kaimen;
    public PathChain fashediyipai;
    public PathChain xidierpai;
    public PathChain fashedierpai;
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
            case 10:
                follower.followPath(fasheyuzhi, true);
                turret.setTargetAngle(-53.0);
                flywheel.setTargetRPM(GlobalConstants.AUTO_RPM_NORMAL);
                intakeShooter.setIntakePower(0.0);
                setPathState(11);
                break;
            case 11:
                if (!follower.isBusy()) {
                    intakeShooter.startPrecisionShoot(GlobalConstants.SHOOT_TIME_NORMAL);
                    setPathState(12);
                }
                break;
            case 12:
                if (!intakeShooter.isShootingActive()) {
                    setPathState(20);
                }
                break;

            case 20:
                pitch.setPitch(GlobalConstants.PITCH_POS_INTAKE_NORMAL);
                flywheel.setTargetRPM(GlobalConstants.AUTO_RPM_DOOR_1);
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
                follower.followPath(kaimen, true);
                intakeShooter.setIntakePower(0.0);
                turret.setTargetAngle(-50.5);
                setPathState(31);
                break;
            case 31:
                if (!follower.isBusy()) {
                    pathTimer.resetTimer();
                    setPathState(32);
                }
                break;
            case 32:
                if (pathTimer.getElapsedTimeSeconds() >= 1.0) {
                    setPathState(40);
                }
                break;

            case 40:
                follower.followPath(fashediyipai, true);
                setPathState(41);
                break;
            case 41:
                if (!follower.isBusy()) {
                    intakeShooter.startPrecisionShoot(GlobalConstants.SHOOT_TIME_NORMAL);
                    setPathState(42);
                }
                break;
            case 42:
                if (!intakeShooter.isShootingActive()) {
                    setPathState(50);
                }
                break;

            case 50:
                follower.followPath(xidierpai, false);
                intakeShooter.setIntakePower(1.0);
                setPathState(51);
                break;
            case 51:
                if (!follower.isBusy()) {
                    setPathState(60);
                }
                break;

            case 60:
                follower.followPath(fashedierpai, true);
                intakeShooter.setIntakePower(0.0);
                setPathState(61);
                break;
            case 61:
                if (!follower.isBusy()) {
                    intakeShooter.startPrecisionShoot(GlobalConstants.SHOOT_TIME_NORMAL);
                    setPathState(62);
                }
                break;
            case 62:
                if (!intakeShooter.isShootingActive()) {
                    setPathState(70);
                }
                break;

            case 70:
                follower.followPath(xidisanpai, false);
                intakeShooter.setIntakePower(1.0);
                setPathState(71);
                break;
            case 71:
                if (!follower.isBusy()) {
                    setPathState(80);
                }
                break;

            case 80:
                follower.followPath(fashedisanpai, true);
                intakeShooter.setIntakePower(0.0);
                setPathState(81);
                break;
            case 81:
                if (!follower.isBusy()) {
                    intakeShooter.startPrecisionShoot(GlobalConstants.SHOOT_TIME_NORMAL);
                    setPathState(82);
                }
                break;
            case 82:
                if (!intakeShooter.isShootingActive()) {
                    setPathState(90);
                }
                break;

            case 90:
                follower.followPath(tingkao, true);
                turret.setTargetAngle(0.0);
                setPathState(91);
                break;
            case 91:
                if (!follower.isBusy()) {
                    setPathState(100);
                }
                break;

            case 100:
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
        telemetry.addData("Turret Target Angle", turret.getTargetAngle());
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