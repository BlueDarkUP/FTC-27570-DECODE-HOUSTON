package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class IntakeSubsystem {

    private DcMotorEx motorIntake;

    private final double STALL_CURRENT_AMPS = 2.45;
    private final double STALL_COOLDOWN_SEC = 0.5;
    private final double STALL_TIME_THRESHOLD_SEC = 0.3;
    private final double CURRENT_READ_INTERVAL = 0.05;

    private double intakeBrakeReleaseTime = 0.0;
    private double stallStartTime = 0.0;
    private boolean isStalling = false;
    private double unwindReverseEndTime = 0.0;
    private boolean wasUnwinding = false;
    private double lastCurrentReadTime = 0.0;
    private double lastMeasuredCurrent = 0.0;

    private ElapsedTime runtime = new ElapsedTime();
    private String systemStatusMessage = "初始化中...";

    public IntakeSubsystem(HardwareMap hardwareMap) {
        motorIntake = hardwareMap.get(DcMotorEx.class, "Intake");
        motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        runtime.reset();
    }

    public void start() {
        runtime.reset();
    }

    public void update(boolean isShootingMode, boolean hasTarget, boolean isUnwinding, boolean isAimLocked, boolean isFlywheelReady, double targetDist, boolean isBBBReady) {
        double currentTime = runtime.seconds();

        if (isShootingMode) {
            isStalling = false;
            lastMeasuredCurrent = 0.0;

            if (hasTarget && isUnwinding) {
                if (!wasUnwinding) unwindReverseEndTime = currentTime + 0.2;
                if (currentTime < unwindReverseEndTime) {
                    motorIntake.setPower(-0.1);
                    systemStatusMessage = "⚠️ 云台复位: 反转退弹";
                } else {
                    motorIntake.setPower(0.0);
                    systemStatusMessage = "⚠️ 云台复位: 刹车";
                }
                intakeBrakeReleaseTime = 0.0;
            } else {
                if (!isBBBReady) {
                    motorIntake.setPower(0.0);
                    systemStatusMessage = "⏳ 等待 BBB 展开 (600ms)...";
                } else if (hasTarget) {
                    if (targetDist < 130.0) {
                        motorIntake.setPower(1.0);
                        systemStatusMessage = "⚡ 跑打强行给弹中 (近战 1.0)!";
                    } else {
                        motorIntake.setPower(0.6);
                        systemStatusMessage = "⚡ 跑打强行给弹中 (远射 0.9)!";
                    }
                } else {
                    motorIntake.setPower(0.0);
                    systemStatusMessage = "射击待机 (无目标)...";
                }
            }
        } else {
            if (currentTime - lastCurrentReadTime >= CURRENT_READ_INTERVAL) {
                lastMeasuredCurrent = motorIntake.getCurrent(CurrentUnit.AMPS);
                lastCurrentReadTime = currentTime;
            }

            if (lastMeasuredCurrent >= STALL_CURRENT_AMPS) {
                if (!isStalling) {
                    isStalling = true;
                    stallStartTime = currentTime;
                } else if (currentTime - stallStartTime >= STALL_TIME_THRESHOLD_SEC) {
                    intakeBrakeReleaseTime = currentTime + STALL_COOLDOWN_SEC;
                }
            } else {
                isStalling = false;
            }

            if (currentTime < intakeBrakeReleaseTime) {
                motorIntake.setPower(0.0);
                systemStatusMessage = "⚠️ 堵转保护冷却";
                isStalling = false;
            } else {
                motorIntake.setPower(1.0);
                systemStatusMessage = "正常收件中";
            }
        }

        wasUnwinding = isUnwinding;
    }

    public String getStatusMessage() {
        return systemStatusMessage;
    }
}