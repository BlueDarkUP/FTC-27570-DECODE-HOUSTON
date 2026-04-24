package org.firstinspires.ftc.teamcode.Auto.Red;

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

@Autonomous(name = "红方近点18球自动", group = "Autonomous")
public class RED23close18 extends OpMode {

    private Follower follower;
    private PitchSubsystem pitch;
    private TurretSubsystem turret;
    private FlywheelSubsystem flywheel;
    private IntakeShooterSubsystem intakeShooter;

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

    private final Pose startPose = new Pose(109.000, 134.600, Math.toRadians(0));

    public void buildPaths() {
        diyigepaoda = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(109.000, 135.000),
                                new Pose(96.000, 86.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        xidierpai = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(96.000, 86.000),
                                new Pose(83.346, 54.923),
                                new Pose(131.000, 59.000)
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        fashedierpai = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(133.000, 59.000), new Pose(99.000, 60.000), new Pose(84.000, 83.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-46))
                .build();

        kaimenzuo = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(84.000, 83.000),
                                new Pose(99.000, 61.3000),
                                new Pose(132.500, 60)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(23))
                .build();

        fashekaimenzuo = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(132.500, 60), new Pose(109.000, 47.000), new Pose(84.000, 83.000)))
                .setLinearHeadingInterpolation(Math.toRadians(23), Math.toRadians(-46))
                .build();

        zhunbeixidiyipai = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(84.000, 83.000), new Pose(99.000, 83.000)))
                .setLinearHeadingInterpolation(Math.toRadians(-46), Math.toRadians(0))
                .setNoDeceleration()
                .build();

        xidiyipai = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(99.000, 83.000), new Pose(125.000, 83.000)))
                .setTangentHeadingInterpolation()
                .build();

        fashediyipai = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(125.000, 83.000), new Pose(84.000, 83.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-50))
                .build();

        zhunbeixidisanpai = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(84.000, 83.000), new Pose(94.000, 36.000)))
                .setLinearHeadingInterpolation(Math.toRadians(-50), Math.toRadians(0))
                .setNoDeceleration()
                .build();

        xidisanpai = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(94.000, 36.000), new Pose(132.000, 36.000)))
                .setTangentHeadingInterpolation()
                .build();

        Returntofixedlaunchpoint = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(131.000, 35.800), new Pose(84.077, 107.923)))
                .setConstantHeadingInterpolation(Math.toRadians(0))
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
                turret.setTargetAngle(46.0);
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
                follower.setMaxPower(0.85);
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
                follower.setMaxPower(1);
                follower.followPath(fashedierpai, true);
                turret.setTargetAngle(94.0);
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
                if (pathTimer.getElapsedTimeSeconds() >= 0) {
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
                if (pathTimer.getElapsedTimeSeconds() >= 1) {
                    setPathState(50);
                }
                break;

            case 50:
                follower.followPath(fashekaimenzuo, true);
                turret.setTargetAngle(90.0);
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
                if (pathTimer.getElapsedTimeSeconds() >= 0) {
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
                turret.setTargetAngle(90.0);
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
                if (pathTimer.getElapsedTimeSeconds() >= 0) {
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
                turret.setTargetAngle(24.0);
                flywheel.setTargetRPM(3050);
                intakeShooter.setIntakePower(0.0);
                setPathState(111);
                break;
            case 111:
                if (!follower.isBusy()) {
                    setPathState(1115);
                }
                break;
            case 1115:
                if (pathTimer.getElapsedTimeSeconds() >= 0) {
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
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        doorExtractLoopCount = 0;
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
    }

    @Override
    public void stop() {
        turret.stop();
        flywheel.stop();
        intakeShooter.stop();
    }
}