package org.firstinspires.ftc.teamcode.AutoAim;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class AutoAimSubsystem {

    private DcMotorEx Turret;
    private Servo LP;
    private Servo RP;

    public static double TURRET_kP = 0.0025;
    public static double TURRET_kI = 0.01;
    public static double TURRET_kD = 0.0001;
    public static double TURRET_kF = 0.0;

    public static double TURRET_DEADZONE_DEG = 0.8;

    public static double TURRET_MAX_POWER = 0.9;

    private PIDFController turretPIDF;
    private final double TICKS_PER_REV = 32798.0;

    private final double LP_UP = 1.0;
    private final double LP_DOWN = 0.4;
    private final double RP_UP = 0.0;
    private final double RP_DOWN = 0.5;

    private final double AIM_LOCK_TOLERANCE_DEG = 1;

    public static class TurretCommand {
        public boolean hasTarget = false;
        public double targetRpm = 0.0;
        public double targetPitch = 0.0;
        public boolean isAimLocked = false;
        public boolean isUnwinding = false;
    }

    public AutoAimSubsystem(HardwareMap hardwareMap) {
        Turret = hardwareMap.get(DcMotorEx.class, "Turret");
        Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        LP = hardwareMap.get(Servo.class, "LP");
        RP = hardwareMap.get(Servo.class, "RP");

        turretPIDF = new PIDFController(TURRET_kP, TURRET_kI, TURRET_kD, TURRET_kF);

        setPitchServos(0.7);
    }

    private void setPitchServos(double targetPitch) {
        double clampedLP = Math.max(LP_DOWN, Math.min(LP_UP, targetPitch));
        double proportion = (clampedLP - LP_DOWN) / (LP_UP - LP_DOWN);
        double calculatedRP = RP_DOWN + proportion * (RP_UP - RP_DOWN);
        calculatedRP = Math.max(0.0, Math.min(1.0, calculatedRP));

        LP.setPosition(clampedLP);
        RP.setPosition(calculatedRP);
    }

    public TurretCommand update(
            double robotX, double robotY, double globalVx, double globalVy, double currentHeadingDeg,
            double targetX, double targetY,
            boolean isManualMode, double manualDist) {

        turretPIDF.setPIDF(TURRET_kP, TURRET_kI, TURRET_kD, TURRET_kF);

        TurretCommand command = new TurretCommand();

        AimCalculator.AimResult aimResult;

        if (isManualMode) {
            double manualRpm = AimCalculator.interpolate(manualDist, 1);
            double manualPitch = AimCalculator.interpolate(manualDist, 2);
            aimResult = new AimCalculator.AimResult(manualDist, currentHeadingDeg, manualRpm, manualPitch, 0.0);
        } else {
            aimResult = AimCalculator.solveAim(
                    robotX, robotY, globalVx, globalVy, 0, 0, targetX, targetY
            );
        }

        if (aimResult != null) {
            command.hasTarget = true;
            command.targetRpm = aimResult.rpm;
            command.targetPitch = aimResult.pitch;

            double currentTurretTicks = Turret.getCurrentPosition();
            double currentTurretRelAngle = (currentTurretTicks / TICKS_PER_REV) * 360.0;
            double currentTurretAbsAngle = currentHeadingDeg + currentTurretRelAngle;

            double error = aimResult.algYaw - currentTurretAbsAngle;

            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            double targetTurretRelAngle = currentTurretRelAngle + error;

            if (targetTurretRelAngle > 175.0) {
                targetTurretRelAngle -= 360.0;
                command.isUnwinding = true;
            } else if (targetTurretRelAngle < -210.0) {
                targetTurretRelAngle += 360.0;
                command.isUnwinding = true;
            } else {
                command.isUnwinding = false;
            }

            command.isAimLocked = Math.abs(error) <= AIM_LOCK_TOLERANCE_DEG;

            double targetTurretTicks = (targetTurretRelAngle / 360.0) * TICKS_PER_REV;
            double turretPower = turretPIDF.calculate(currentTurretTicks, targetTurretTicks);

            if (Math.abs(error) < TURRET_DEADZONE_DEG) {
                turretPower = 0.0;
                turretPIDF.reset();
            }

            turretPower = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, turretPower));

            Turret.setPower(turretPower);

            setPitchServos(command.targetPitch);
        } else {
            Turret.setPower(0);
        }

        return command;
    }

    public void stop() {
        Turret.setPower(0);
    }
}