package org.firstinspires.ftc.teamcode.AutoAim.Test;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name = "Limelight MT2 Auto Localization with Pinpoint", group = "Sensor")
public class LimelightMT2Test extends LinearOpMode {

    private Limelight3A limelight;
    private GoBildaPinpointDriver pinpoint;

    @Override
    public void runOpMode() throws InterruptedException {

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        pinpoint.resetPosAndIMU();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.setPollRateHz(100);

        limelight.start();

        telemetry.addLine("初始化完成");
        telemetry.addLine("当前使用 Pinpoint 的精准 Yaw 驱动 Limelight MT2");
        telemetry.addLine("请确保 Limelight 正在运行 AprilTag 管道 (Pipeline)");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            pinpoint.update();

            Pose2D pinpointPose = pinpoint.getPosition();
            double robotYaw = pinpointPose.getHeading(AngleUnit.DEGREES);

            limelight.updateRobotOrientation(robotYaw);

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                Pose3D botposeMT2 = result.getBotpose_MT2();

                if (botposeMT2 != null) {
                    telemetry.addLine("--- Limelight MT2 数据有效 ---");
                    telemetry.addData("MT2完整位姿", botposeMT2.toString());

                    telemetry.addData("X (Meters)", "%.3f", botposeMT2.getPosition().x);
                    telemetry.addData("Y (Meters)", "%.3f", botposeMT2.getPosition().y);
                    telemetry.addData("Z (Meters)", "%.3f", botposeMT2.getPosition().z);
                } else {
                    telemetry.addLine("--- 未获取到 MT2 位姿 (视野内可能无 AprilTag) ---");
                }
            } else {
                telemetry.addLine("--- Limelight 无有效识别目标 ---");
            }

            telemetry.addData("当前 Pinpoint Yaw (度)", "%.2f", robotYaw);
            telemetry.addData("Pinpoint 设备状态", pinpoint.getDeviceStatus().toString());

            telemetry.update();
        }

        limelight.stop();
    }
}