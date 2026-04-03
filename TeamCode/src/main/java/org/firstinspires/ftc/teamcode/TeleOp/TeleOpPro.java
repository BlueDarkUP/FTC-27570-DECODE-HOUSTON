package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.AutoAim.AutoAimSubsystem;

@TeleOp(name="Mecanum Drive TeleOp", group="Linear Opmode")
public class TeleOpPro extends LinearOpMode {

    private AutoAimSubsystem autoAim;

    private DcMotor frontLeftMotor = null;
    private DcMotor backLeftMotor = null;
    private DcMotor frontRightMotor = null;
    private DcMotor backRightMotor = null;

    final double TARGET_X_WORLD = 132.0;
    final double TARGET_Y_WORLD = 136.0;

    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing Hardware, Please wait...");
        telemetry.update();

        frontLeftMotor  = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        backLeftMotor   = hardwareMap.get(DcMotor.class, "backLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backRightMotor  = hardwareMap.get(DcMotor.class, "backRightMotor");

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 初始化子系统
        autoAim = new AutoAimSubsystem(hardwareMap, telemetry);

        sleep(500); // 留给 Pinpoint 清零的时间（合理）

        Pose2D startPose = new Pose2D(DistanceUnit.INCH, 72, 72, AngleUnit.DEGREES, 0);
        autoAim.setInitialPose(startPose);

        // ================= INIT 阶段循环 =================
        while (opModeInInit()) {
            // ⚠️ 传入 false：只刷新定位和视觉计算，不给云台电机通电！安全第一！
            autoAim.update(TARGET_X_WORLD, TARGET_Y_WORLD);

            telemetry.addLine("Ready to Start - Odometry is Active, Turret is SAFELY LOCKED.");
            telemetry.update();
        }

        waitForStart();

        // ================= START 阶段循环 =================
        while (opModeIsActive()) {

            // 🎯 传入 true：比赛正式开始，允许云台电机通电追踪目标！
            AutoAimSubsystem.TurretCommand command = autoAim.update(TARGET_X_WORLD, TARGET_Y_WORLD);

            if (command.hasTarget) {
                // shooterMotor.setVelocity(command.targetRpm);
                // pitchServo.setPosition(command.targetPitch);
            } else {
                // shooterMotor.setVelocity(0);
            }

            // --- 底盘控制逻辑 ---
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1; // 摩擦力补偿👍
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            // 调试输出（AutoAim 的 telemetry 会在 update 内部自动调用 addData，这里只需要补底盘的即可）
            telemetry.addData("Drive Power", "FL:%.2f FR:%.2f BL:%.2f BR:%.2f",
                    frontLeftPower, frontRightPower, backLeftPower, backRightPower);
            telemetry.update();
        }
    }
}