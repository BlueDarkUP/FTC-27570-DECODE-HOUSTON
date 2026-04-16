package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MecanumDriveSubsystem {

    private DcMotor lf, rf, lb, rb;

    public MecanumDriveSubsystem(HardwareMap hardwareMap) {
        lf = hardwareMap.get(DcMotor.class, "lf");
        rf = hardwareMap.get(DcMotor.class, "rf");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");

        lf.setDirection(DcMotor.Direction.REVERSE);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rb.setDirection(DcMotor.Direction.FORWARD);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void driveRobotCentric(double x, double y, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        lf.setPower((y + x + rx) / denominator);
        lb.setPower((y - x + rx) / denominator);
        rf.setPower((y - x - rx) / denominator);
        rb.setPower((y + x - rx) / denominator);
    }

    public void driveFieldCentric(double x, double y, double rx, double headingDegrees) {
        double currentHeadingRad = Math.toRadians(headingDegrees);

        // 向量旋转计算
        double rotX = x * Math.cos(-currentHeadingRad) - y * Math.sin(-currentHeadingRad);
        double rotY = x * Math.sin(-currentHeadingRad) + y * Math.cos(-currentHeadingRad);

        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);

        lf.setPower((rotY + rotX + rx) / denominator);
        lb.setPower((rotY - rotX + rx) / denominator);
        rf.setPower((rotY - rotX - rx) / denominator);
        rb.setPower((rotY + rotX - rx) / denominator);
    }

    public void stop() {
        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
    }
}