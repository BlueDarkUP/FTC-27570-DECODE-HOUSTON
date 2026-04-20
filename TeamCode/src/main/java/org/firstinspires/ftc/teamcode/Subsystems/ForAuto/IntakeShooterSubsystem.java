package org.firstinspires.ftc.teamcode.Subsystems.ForAuto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GlobalConstants;

public class IntakeShooterSubsystem {
    public enum ShootMode { NONE, PRECISION, BLIND }

    private DcMotorEx intakeMotor;
    private Servo bbb;

    private ShootMode currentShootMode = ShootMode.NONE;
    private ElapsedTime shootTimer;
    private double activeShootDuration = 0.0;

    public IntakeShooterSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "Intake");
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        bbb = hardwareMap.get(Servo.class, "bbb");
        bbb.setPosition(GlobalConstants.BBB_IDLE_POS);

        shootTimer = new ElapsedTime();
    }

    public void startPrecisionShoot(double durationSeconds) {
        currentShootMode = ShootMode.PRECISION;
        activeShootDuration = durationSeconds;
        shootTimer.reset();
    }

    public void startBlindShoot(double durationSeconds) {
        currentShootMode = ShootMode.BLIND;
        activeShootDuration = durationSeconds;
        shootTimer.reset();
    }

    public boolean isShootingActive() {
        return currentShootMode != ShootMode.NONE;
    }

    public ShootMode getCurrentShootMode() {
        return currentShootMode;
    }

    public void setIntakePower(double power) {
        intakeMotor.setPower(power);
    }

    public void setBBServo(double position) {
        bbb.setPosition(position);
    }

    public void update(FlywheelSubsystem flywheel) {
        if (currentShootMode == ShootMode.NONE) {
            return;
        }

        if (shootTimer.seconds() >= activeShootDuration) {
            currentShootMode = ShootMode.NONE;
            bbb.setPosition(GlobalConstants.BBB_IDLE_POS);
            intakeMotor.setPower(0.0);
            return;
        }

        if (currentShootMode == ShootMode.BLIND) {
            bbb.setPosition(GlobalConstants.BBB_SHOOT_POS);
            intakeMotor.setPower(1.0);
        }
        else if (currentShootMode == ShootMode.PRECISION) {
            double errorRPM = Math.abs(flywheel.getTargetRPM() - flywheel.getCurrentRPM());
            if (errorRPM <= 500) {
                bbb.setPosition(GlobalConstants.BBB_SHOOT_POS);
                intakeMotor.setPower(1.0);
            } else {
                bbb.setPosition(GlobalConstants.BBB_IDLE_POS);
                intakeMotor.setPower(0.0);
            }
        }
    }

    public void stop() {
        intakeMotor.setPower(0);
        bbb.setPosition(GlobalConstants.BBB_IDLE_POS);
    }
}