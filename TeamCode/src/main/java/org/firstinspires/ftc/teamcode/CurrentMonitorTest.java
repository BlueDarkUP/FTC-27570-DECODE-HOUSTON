package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Current Monitor Test", group = "Test")
public class CurrentMonitorTest extends LinearOpMode {

    // 1. 声明 DcMotorEx 对象（注意是 Ex 结尾）
    private DcMotorEx motorHS;
    private DcMotorEx motorSH;

    @Override
    public void runOpMode() {
        // 2. 硬件映射
        // 确保 Driver Station 里的配置文件名字也是 "HS" 和 "SH"
        motorHS = hardwareMap.get(DcMotorEx.class, "HS");
        motorSH = hardwareMap.get(DcMotorEx.class, "SH");

        // 设置电机模式（可选）
        motorHS.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSH.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 重置编码器（可选，好习惯）
        motorHS.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSH.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorHS.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorSH.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("Status", "Waiting for Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 3. 简单的控制逻辑，让电机动起来产生电流
            // gamepad1 左摇杆控制 HS，右摇杆控制 SH
            double powerHS = -gamepad1.left_stick_y;
            double powerSH = -gamepad1.right_stick_y;

            motorHS.setPower(powerHS);
            motorSH.setPower(powerSH);

            // 4. 获取电流的核心代码
            // CurrentUnit.AMPS 表示单位是安培，MILLIAMPS 是毫安
            double currentHS = motorHS.getCurrent(CurrentUnit.AMPS);
            double currentSH = motorSH.getCurrent(CurrentUnit.AMPS);

            // 5. 将数据显示在 Driver Station 屏幕上
            telemetry.addData(">>> Motor HS <<<", "");
            telemetry.addData("Power", "%.2f", powerHS);
            telemetry.addData("Current", "%.3f Amps", currentHS);

            telemetry.addData("\n>>> Motor SH <<<", "");
            telemetry.addData("Power", "%.2f", powerSH);
            telemetry.addData("Current", "%.3f Amps", currentSH);

            // 这是一个典型的大电流警告逻辑（可选）
            if (currentHS > 8.0 || currentSH > 8.0) {
                telemetry.addData("WARNING", "HIGH CURRENT DETECTED!");
            }

            telemetry.update();
        }
    }
}