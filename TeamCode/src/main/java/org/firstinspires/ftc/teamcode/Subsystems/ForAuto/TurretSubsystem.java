package org.firstinspires.ftc.teamcode.Subsystems.ForAuto;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TurretSubsystem {
    private DcMotorEx turretMotor;

    private final double STAGE_THRESHOLD = 14.0;
    private final double kP_far = 0.05;
    private final double kI_far = 0.00;
    private final double kD_far = 0.0003;

    private final double kP_near = 0.0000002;
    private final double kI_near = 0.0001;
    private final double kD_near = 0.0015;

    private final double kS_near = 0.28;
    private final double I_ZONE_near = 3.0;
    private final double MAX_INTEGRAL_POWER = 0.08;

    private final double ERROR_TOLERANCE = 1.5;
    private final double TURRET_TICKS_PER_REV = 32768;
    private final double TICKS_PER_DEGREE = TURRET_TICKS_PER_REV / 360.0;

    private double integralSum = 0;
    private double lastError = 0;
    private boolean wasInStage1 = false;
    private ElapsedTime pidTimer;

    private double targetAngle = 0.0;

    public TurretSubsystem(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "Turret");
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pidTimer = new ElapsedTime();
        resetTimer();
    }

    public void setTargetAngle(double angle) {
        this.targetAngle = angle;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public void resetTimer() {
        pidTimer.reset();
        integralSum = 0;
        lastError = 0;
    }

    public void update() {
        double currentTicks = turretMotor.getCurrentPosition();
        double currentAngle = currentTicks / TICKS_PER_DEGREE;

        double error = targetAngle - currentAngle;
        double absError = Math.abs(error);

        double dt = pidTimer.seconds();
        if (dt == 0) dt = 0.001;

        double power = 0.0;

        if (absError <= ERROR_TOLERANCE) {
            power = 0;
            integralSum = 0;
            wasInStage1 = false;
        } else if (absError > STAGE_THRESHOLD) {
            wasInStage1 = true;
            integralSum += error * dt;
            double derivative = (error - lastError) / dt;
            power = (kP_far * error) + (kI_far * integralSum) + (kD_far * derivative);
            power = Math.max(-1.0, Math.min(1.0, power));
        } else {
            if (wasInStage1 || Math.signum(error) != Math.signum(lastError)) {
                integralSum = 0;
                wasInStage1 = false;
            }
            if (absError < I_ZONE_near) {
                integralSum += error * dt;
                double maxISum = MAX_INTEGRAL_POWER / (kI_near == 0 ? 1 : kI_near);
                integralSum = Math.max(-maxISum, Math.min(maxISum, integralSum));
            } else {
                integralSum = 0;
            }
            double derivative = (error - lastError) / dt;
            double pidPower = (kP_near * error) + (kI_near * integralSum) + (kD_near * derivative);
            double feedforward = Math.signum(error) * kS_near;

            power = pidPower + feedforward;
            power = Math.max(-1.0, Math.min(1.0, power));
        }

        turretMotor.setPower(power);
        lastError = error;
        pidTimer.reset();
    }

    public void stop() {
        turretMotor.setPower(0);
    }
}