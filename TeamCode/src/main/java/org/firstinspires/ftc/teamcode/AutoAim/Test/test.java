package org.firstinspires.ftc.teamcode.AutoAim.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Only_Climb_Test", group = "TeleOp")
public class test extends LinearOpMode {

    private DcMotor fl, bl, fr, br;
    private Servo archi, medes;

    private IMU imu;

    private boolean isPtoUnlocked = false;
    private boolean lastLBumper = false;
    private ElapsedTime climbTimer = new ElapsedTime();

    private static final double ROLL_TOLERANCE = 2.0;

    @Override
    public void runOpMode() throws InterruptedException {
        fl = hardwareMap.get(DcMotor.class, "lf");
        bl = hardwareMap.get(DcMotor.class, "lb");
        fr = hardwareMap.get(DcMotor.class, "rf");
        br = hardwareMap.get(DcMotor.class, "rb");

        archi = hardwareMap.get(Servo.class, "Archi");
        medes = hardwareMap.get(Servo.class, "Medes");

        imu = hardwareMap.get(IMU.class, "imu");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.FORWARD);
        br.setDirection(DcMotorSimple.Direction.FORWARD);

        DcMotor[] allMotors = {fl, bl, fr, br};
        for (DcMotor m : allMotors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        archi.setPosition(0.07);
        medes.setPosition(1.0);

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);
        imu.resetYaw(); // 可选：重置偏航角

        telemetry.addLine("纯爬升程序初始化完成！");
        telemetry.addLine("Touchpad (按住): 解锁PTO并【自动平衡】爬升");
        telemetry.addLine("LB: 紧急手动复位PTO");
        telemetry.update();

        waitForStart();
        climbTimer.reset();

        while (opModeIsActive()) {

            if (gamepad1.left_bumper && !lastLBumper) {
                isPtoUnlocked = !isPtoUnlocked;
                if (isPtoUnlocked) {
                    climbTimer.reset();
                }
            }
            lastLBumper = gamepad1.left_bumper;

            if (isPtoUnlocked) {
                archi.setPosition(0.45);
                medes.setPosition(0.55);
            } else {
                archi.setPosition(0.07);
                medes.setPosition(1.0);
            }

            double flP = 0, frP = 0, blP = 0, brP = 0;
            double currentRoll = 0;

            if (gamepad1.touchpad) {
                if (!isPtoUnlocked) {
                    isPtoUnlocked = true;
                    climbTimer.reset();
                }

                if (climbTimer.milliseconds() >= 300) {

                    currentRoll = imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);

                    if (currentRoll > ROLL_TOLERANCE) {
                        flP = 0.0;  blP = 0.0;
                        frP = 0.75; brP = -0.75;
                        telemetry.addData("Climb Mode", "Leveling: Right Side Catching Up");
                    } else if (currentRoll < -ROLL_TOLERANCE) {
                        flP = 0.75; blP = -0.75;
                        frP = 0.0;  brP = 0.0;
                        telemetry.addData("Climb Mode", "Leveling: Left Side Catching Up");
                    } else {
                        flP = 0.75; frP = 0.75; blP = -0.75; brP = -0.75;
                        telemetry.addData("Climb Mode", "Balanced Climbing");
                    }

                } else {
                    flP = 0.0; frP = 0.0; blP = 0.0; brP = 0.0;
                    telemetry.addData("Climb Mode", "Waiting for PTO");
                }
            } else {
                double y = -gamepad1.left_stick_y;
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;
                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
                flP = (y + x + rx) / denominator;
                blP = (y - x + rx) / denominator;
                frP = (y - x - rx) / denominator;
                brP = (y + x - rx) / denominator;
                telemetry.addData("Climb Mode", "Not Climbing");
                currentRoll = imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);
            }

            fl.setPower(flP); bl.setPower(blP); fr.setPower(frP); br.setPower(brP);

            telemetry.addData("PTO State", isPtoUnlocked ? "UNLOCKED" : "LOCKED");
            telemetry.addData("Hub IMU Roll", "%.2f degrees", currentRoll);
            telemetry.addData("Motor Power", "L(%.2f) R(%.2f)", flP, frP);
            telemetry.update();
        }
    }
}