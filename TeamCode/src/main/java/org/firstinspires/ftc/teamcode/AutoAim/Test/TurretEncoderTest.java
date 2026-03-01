package org.firstinspires.ftc.teamcode.AutoAim.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="TEST 2: Turret Encoder Reader", group="Test")
public class TurretEncoderTest extends LinearOpMode {

    private DcMotorEx turretEncoder;

    @Override
    public void runOpMode() {
        // 获取硬件 (名字和你的 AutoAimSubsystem 里一致)
        turretEncoder = hardwareMap.get(DcMotorEx.class, "turretEncoder");

        // 停止电机并重置编码器归零，之后设置为无编码器模式跑（纯给电压）
        turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // 可选：如果你的云台电机需要反转
        // turretEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Turret Encoder Ready.");
        telemetry.addLine("Use Gamepad 1 RIGHT STICK (X-axis) to rotate Turret slowly.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // 使用右摇杆 X 轴手动转动云台（给了 30% 限速，防止转太快扯断线）
            double manualPower = gamepad1.right_stick_x * 0.3;
            turretEncoder.setPower(manualPower);

            // 读取绝对 Tick 数量
            int currentTicks = turretEncoder.getCurrentPosition();

            // 假设我们用常用的 8192 做计算参考
            double approxDegrees = (currentTicks / 8192.0) * 360.0;

            telemetry.addLine("--- TURRET ENCODER DATA ---");
            telemetry.addData("Raw Ticks", currentTicks);
            telemetry.addData("Motor Power", "%.2f", manualPower);
            telemetry.addLine();
            telemetry.addData("Approx Degrees (if 8192 CPR)", "%.1f°", approxDegrees);

            telemetry.addLine("\n--- HOW TO TEST TICKS PER REV ---");
            telemetry.addLine("1. Mark a physical starting line on the turret.");
            telemetry.addLine("2. Rotate exactly 1 full rotation (360 degrees).");
            telemetry.addLine("3. Read 'Raw Ticks'. That is your TURRET_TICKS_PER_REV.");
            telemetry.addLine("4. Rotate 10 times and divide by 10 for higher accuracy.");

            telemetry.update();
        }
    }
}