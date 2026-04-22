package org.firstinspires.ftc.teamcode.DeadEye;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class LimelightPinpointLocalizer {

    private final Limelight3A limelight;

    private final double METERS_TO_INCHES = 39.3701;
    private final double HEADING_OFFSET = 180.0;
    private final double HALF_FIELD_INCHES = 72.0;

    public LimelightPinpointLocalizer(HardwareMap hardwareMap, String deviceName) {
        limelight = hardwareMap.get(Limelight3A.class, deviceName);
        limelight.setPollRateHz(100);
    }

    public void start() { limelight.start(); }
    public void stop() { limelight.stop(); }
    public Limelight3A getLimelight() { return limelight; }

    public void updateLimelightOrientation(double currentPinpointHeadingDegrees) {
        double limelightYaw = currentPinpointHeadingDegrees + HEADING_OFFSET;

        while (limelightYaw > 180.0) limelightYaw -= 360.0;
        while (limelightYaw <= -180.0) limelightYaw += 360.0;

        limelight.updateRobotOrientation(limelightYaw);
    }

    public Pose2D getTransformedPose(double currentPinpointHeadingDegrees) {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            Pose3D botposeMT2 = result.getBotpose_MT2();

            if (botposeMT2 != null) {
                double rawX_m = botposeMT2.getPosition().x;
                double rawY_m = botposeMT2.getPosition().y;

                double transformedX_in = (-rawX_m * METERS_TO_INCHES) + HALF_FIELD_INCHES;
                double transformedY_in = (-rawY_m * METERS_TO_INCHES) + HALF_FIELD_INCHES;

                return new Pose2D(
                        DistanceUnit.INCH, transformedX_in, transformedY_in,
                        AngleUnit.DEGREES, currentPinpointHeadingDegrees
                );
            }
        }
        return null;
    }
}