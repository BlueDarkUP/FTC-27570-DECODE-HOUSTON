package org.firstinspires.ftc.teamcode.AutoAim.Test;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@TeleOp(name="DEBUG: Limelight Coordinate Mapper", group="Test")
public class LimelightCoordinateMapperTestttt extends LinearOpMode {

    private Limelight3A ll;
    private GoBildaPinpointDriver odo;

    final double FIELD_OFFSET_X = 72.0;
    final double FIELD_OFFSET_Y = 72.0;

    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing Limelight and Pinpoint...");
        telemetry.update();

        try {
            ll = hardwareMap.get(Limelight3A.class, "limelight");
            ll.pipelineSwitch(0);
            ll.start();
            telemetry.addLine("[OK] Limelight Ready.");
        } catch (Exception e) {
            telemetry.addLine("[FATAL ERROR] Limelight not found in hardwareMap!");
        }

        try {
            odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
            odo.resetPosAndIMU();
            odo.recalibrateIMU();
            telemetry.addLine("[OK] goBILDA Pinpoint (odo) Ready.");
        } catch (Exception e) {
            telemetry.addLine("[FATAL ERROR] Pinpoint 'odo' not found in hardwareMap!");
        }

        telemetry.addLine("Press START to begin coordinate mapping test with MegaTag2.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double rawOdoYaw = 0.0;
            double llInputYaw = 0.0;

            if (odo != null) {
                odo.update();
                rawOdoYaw = odo.getPosition().getHeading(AngleUnit.DEGREES);

                llInputYaw = AngleUnit.normalizeDegrees(rawOdoYaw+180);
            }

            if (ll != null) {
                ll.updateRobotOrientation(llInputYaw);

                LLResult result = ll.getLatestResult();

                if (result != null && result.isValid()) {
                    Pose3D botpose = result.getBotpose_MT2();

                    if (botpose != null) {
                        double llRawX_Meters = botpose.getPosition().x;
                        double llRawY_Meters = botpose.getPosition().y;
                        double llRawYaw_Deg = botpose.getOrientation().getYaw(AngleUnit.DEGREES);

                        double targetWorldX_Inches = (-llRawX_Meters * 39.3701) + FIELD_OFFSET_X;

                        double targetWorldY_Inches = (-llRawY_Meters * 39.3701) + FIELD_OFFSET_Y;

                        double targetHeading_Deg = AngleUnit.normalizeDegrees(llRawYaw_Deg + 180.0);

                        telemetry.addLine("--- ODOMETRY DATA ---");
                        telemetry.addData("Raw Odo Yaw", "%.1f°", rawOdoYaw);
                        telemetry.addData("Yaw Provided to LL (-90°)", "%.1f°", llInputYaw);

                        telemetry.addLine("\n--- LIMELIGHT MEGATAG2 RAW DATA (Meters, Center=0) ---");
                        telemetry.addData("Raw X (+Rear)   ", "%.3f m", llRawX_Meters);
                        telemetry.addData("Raw Y (+Right)  ", "%.3f m", llRawY_Meters);
                        telemetry.addData("Raw Yaw (0=Rear)", "%.1f°", llRawYaw_Deg);

                        telemetry.addLine("\n--- TARGET FIELD DATA (Inches, Bottom-Right=0) ---");
                        telemetry.addData("Mapped X (+Front)", "%.1f in", targetWorldX_Inches);
                        telemetry.addData("Mapped Y (+Left)", "%.1f in", targetWorldY_Inches);
                        telemetry.addData("Mapped Yaw(0=Front)", "%.1f°", targetHeading_Deg);

                        String direction = "Unknown";
                        if (Math.abs(targetHeading_Deg) <= 30) direction = "⬆️ FRONT (0°)";
                        else if (Math.abs(targetHeading_Deg) >= 150) direction = "⬇️ REAR (180°)";
                        else if (targetHeading_Deg > 30 && targetHeading_Deg < 150) direction = "⬅️ LEFT (+90°)";
                        else if (targetHeading_Deg < -30 && targetHeading_Deg > -150) direction = "➡️ RIGHT (-90°)";

                        telemetry.addLine("\n--- VISUAL COMPASS ---");
                        telemetry.addData("Robot is pointing", direction);
                    } else {
                        telemetry.addLine("AprilTag detected, but MegaTag2 Pose is null.");
                        telemetry.addLine("Check Limelight Web UI configuration.");
                    }

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