package org.firstinspires.ftc.teamcode.vision.EchoLapse;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class PinpointPoseProvider implements IPoseProvider {

    private final GoBildaPinpointDriver odo;
    private Pose2D currentPose;

    public PinpointPoseProvider(HardwareMap hardwareMap, String deviceName) {
        odo = hardwareMap.get(GoBildaPinpointDriver.class, deviceName);
    }

    public void initialize() {
        // 根据您的机器人配置进行设置
        odo.setOffsets(160,-75,DistanceUnit.MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        reset(); // 初始重置
    }

    @Override
    public void update() {
        odo.update();
        currentPose = odo.getPosition();
    }

    @Override
    public double getX(DistanceUnit unit) {
        return currentPose != null ? currentPose.getY(unit) : 0;
    }

    @Override
    public double getY(DistanceUnit unit) {
        return currentPose != null ? currentPose.getX(unit) : 0;
    }

    @Override
    public double getHeading(AngleUnit unit) {
        return currentPose != null ? currentPose.getHeading(unit) : 0;
    }

    @Override
    public double getXVelocity(DistanceUnit unit) {
        return odo.getVelY(unit);
    }

    @Override
    public double getYVelocity(DistanceUnit unit) {
        return odo.getVelX(unit);
    }

    @Override
    public double getHeadingVelocity(AngleUnit unit) {
        double headingVelocityInDegrees = odo.getHeadingVelocity(UnnormalizedAngleUnit.DEGREES);
        if (unit == AngleUnit.RADIANS) {
            return Math.toRadians(headingVelocityInDegrees);
        } else {
            return headingVelocityInDegrees;
        }
    }

    @Override
    public void reset() {
        odo.resetPosAndIMU();
    }

    public void recalibrateIMU() {
        odo.recalibrateIMU();
    }

    @Override
    public String getDeviceStatus() {
        return odo.getDeviceStatus().toString();
    }

    @Override
    public double getUpdateFrequency() {
        return odo.getFrequency();
    }

    /**
     * 使用一个标准的场地坐标系Pose2D来强制重置Pinpoint的内部位置。
     * 这个方法会处理所有必要的坐标系转换。
     * @param fieldPose 从AprilTag等外部传感器获取的，以场地标准坐标系定义的Pose2D对象。
     *                  (+Y 向前, +X 向右)
     */
    public void setPose(Pose2D fieldPose) {
        if (fieldPose == null) {
            return;
        }
        Pose2D poseForDriver = new Pose2D(
                DistanceUnit.MM,                                  // 明确指定距离单位
                fieldPose.getY(DistanceUnit.MM),                  // Pinpoint的X(向前)应设为场地的Y(向前)
                -fieldPose.getX(DistanceUnit.MM),                 // Pinpoint的Y(向左)应设为场地的-X(向右)
                AngleUnit.RADIANS,                                // 明确指定角度单位
                fieldPose.getHeading(AngleUnit.RADIANS)           // 航向角单位一致
        );

        odo.setPosition(poseForDriver);
        update();
    }
}
