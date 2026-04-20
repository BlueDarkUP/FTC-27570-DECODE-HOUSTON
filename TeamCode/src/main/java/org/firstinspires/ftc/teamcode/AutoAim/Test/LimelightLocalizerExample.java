package org.firstinspires.ftc.teamcode.AutoAim.Test;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.DeadEye.LimelightPinpointLocalizer;

@TeleOp(name = "Limelight Wrapper Example", group = "Sensor")
public class LimelightLocalizerExample extends LinearOpMode {

    private GoBildaPinpointDriver pinpoint;

    // 声明我们刚刚封装好的工具类
    private LimelightPinpointLocalizer visionLocalizer;

    @Override
    public void runOpMode() throws InterruptedException {

        // 1. 初始化 Pinpoint 里程计
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        pinpoint.resetPosAndIMU();

        // 2. 初始化封装的 Limelight 工具类
        visionLocalizer = new LimelightPinpointLocalizer(hardwareMap, "limelight");
        visionLocalizer.start();

        telemetry.addLine("初始化完成");
        telemetry.addLine("提示: 在运行期间按下 Gamepad 1 的 'A' 键即可用视觉数据重置 Pinpoint 位置");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // 更新里程计
            pinpoint.update();
            Pose2D pinpointPose = pinpoint.getPosition();
            double currentHeading = pinpointPose.getHeading(AngleUnit.DEGREES);

            Pose2D visionPose = visionLocalizer.getTransformedPose(currentHeading);

            if (visionPose != null) {
                telemetry.addLine("--- 视觉定位已锁定 ---");
                telemetry.addData("Vision X (in)", "%.2f", visionPose.getX(DistanceUnit.INCH));
                telemetry.addData("Vision Y (in)", "%.2f", visionPose.getY(DistanceUnit.INCH));

                if (gamepad1.a) {
                    pinpoint.setPosition(visionPose);
                    telemetry.addLine(">>>> Pinpoint 坐标已根据视觉重置！ <<<<");
                }
            } else {
                telemetry.addLine("--- 视野内无目标 (依赖 Pinpoint 推算) ---");
            }

            telemetry.addLine("\n--- 当前 Pinpoint 数据 ---");
            telemetry.addData("Odo X (in)", "%.2f", pinpointPose.getX(DistanceUnit.INCH));
            telemetry.addData("Odo Y (in)", "%.2f", pinpointPose.getY(DistanceUnit.INCH));
            telemetry.addData("Heading (Deg)", "%.2f", currentHeading);

            telemetry.update();
        }

        visionLocalizer.stop();
    }
}