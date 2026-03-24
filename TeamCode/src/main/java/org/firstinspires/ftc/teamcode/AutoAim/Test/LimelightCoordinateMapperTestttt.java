package org.firstinspires.ftc.teamcode.AutoAim.Test;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name="DEBUG: Limelight Coordinate Mapperfff", group="Test")
public class LimelightCoordinateMapperTestttt extends LinearOpMode {

    private Limelight3A ll;

    // 场地的平移偏移量 (将中心 0,0 移到左下角)
    final double FIELD_OFFSET_X = 72.0;
    final double FIELD_OFFSET_Y = 72.0;

    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing Limelight...");
        telemetry.update();

        try {
            ll = hardwareMap.get(Limelight3A.class, "limelight");
            ll.pipelineSwitch(0); // 确保切换到包含 AprilTag 3D 定位的 Pipeline
            ll.start();
            telemetry.addLine("[OK] Limelight Ready.");
            telemetry.addLine("Press START to begin coordinate mapping test.");
        } catch (Exception e) {
            telemetry.addLine("[FATAL ERROR] Limelight not found in hardwareMap!");
        }
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (ll != null) {
                LLResult result = ll.getLatestResult();

                if (result != null && result.isValid()) {
                    Pose3D botpose = result.getBotpose();

                    // ==========================================
                    // 1. 获取 Limelight 原始数据 (中心原点，单位：米)
                    // ==========================================
                    double llRawX_Meters = botpose.getPosition().x;
                    double llRawY_Meters = botpose.getPosition().y;
                    double llRawYaw_Deg = botpose.getOrientation().getYaw(AngleUnit.DEGREES);

                    // ==========================================
                    // 2. 坐标系洗牌与映射 (底层核心测试逻辑)
                    // ==========================================
                    // 根据实测定义：
                    // LL 的 +X 指向场地后方 (-Y 目标)
                    // LL 的 +Y 指向场地右侧 (+X 目标)

                    // X轴映射：将 LL 的 Y (米) 转换为 英寸，并加上 X 的偏移量
                    double targetWorldX_Inches = (llRawY_Meters * 39.3701) + FIELD_OFFSET_X;

                    // Y轴映射：将 LL 的 X 取反转为正前方方向 (米) 转换为 英寸，加上 Y 偏移量
                    double targetWorldY_Inches = (-llRawX_Meters * 39.3701) + FIELD_OFFSET_Y;

                    // 航向角映射：LL 的 0度在后方，我们期望的 0度在正前方。直接相差 180 度。
                    // 加上 180 度后进行标准化，保证输出在 [-180, 180] 之间
                    double targetHeading_Deg = AngleUnit.normalizeDegrees(llRawYaw_Deg + 180.0);


                    // ==========================================
                    // 3. UI 显示与测试指引
                    // ==========================================
                    telemetry.addLine("--- LIMELIGHT RAW DATA (Meters, Center=0) ---");
                    telemetry.addData("Raw X (+Rear)   ", "%.3f m", llRawX_Meters);
                    telemetry.addData("Raw Y (+Right)  ", "%.3f m", llRawY_Meters);
                    telemetry.addData("Raw Yaw (0=Rear)", "%.1f°", llRawYaw_Deg);

                    telemetry.addLine("\n--- TARGET FIELD DATA (Inches, Corner=0) ---");
                    telemetry.addData("Mapped X (+Right)", "%.1f in", targetWorldX_Inches);
                    telemetry.addData("Mapped Y (+Front)", "%.1f in", targetWorldY_Inches);
                    telemetry.addData("Mapped Yaw(0=Front)", "%.1f°", targetHeading_Deg);

                    // 增加一个直观的罗盘提示，方便你脱离数字直接判断
                    String direction = "Unknown";
                    if (Math.abs(targetHeading_Deg) <= 30) direction = "⬆️ FRONT (0°)";
                    else if (Math.abs(targetHeading_Deg) >= 150) direction = "⬇️ REAR (180°)";
                    else if (targetHeading_Deg > 30 && targetHeading_Deg < 150) direction = "⬅️ LEFT (+90°)";
                    else if (targetHeading_Deg < -30 && targetHeading_Deg > -150) direction = "➡️ RIGHT (-90°)";

                    telemetry.addLine("\n--- VISUAL COMPASS ---");
                    telemetry.addData("Robot is pointing", direction);

                } else {
                    telemetry.addLine("Limelight is running but NO AprilTag detected.");
                    telemetry.addLine("Please point the camera at a field AprilTag.");
                }
            }
            telemetry.update();
        }

        if (ll != null) {
            ll.stop();
        }
    }
}