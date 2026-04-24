package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class ClimbSubsystem {
    private DcMotor fl, bl, fr, br;
    private Servo archi, medes;
    private IMU imu;

    private boolean isPtoUnlocked = false;
    private ElapsedTime climbTimer = new ElapsedTime();
    private static final double ROLL_TOLERANCE = 2.0;

    public void init(HardwareMap hardwareMap) {
        // 获取底盘电机
        fl = hardwareMap.get(DcMotor.class, "lf");
        bl = hardwareMap.get(DcMotor.class, "lb");
        fr = hardwareMap.get(DcMotor.class, "rf");
        br = hardwareMap.get(DcMotor.class, "rb");
        archi = hardwareMap.get(Servo.class, "Archi");
        medes = hardwareMap.get(Servo.class, "Medes");
        imu = hardwareMap.get(IMU.class, "imu");

        lockPto();
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP
        ));
        imu.initialize(parameters);
        imu.resetYaw();
    }

    public void togglePto() {
        isPtoUnlocked = !isPtoUnlocked;
        if (isPtoUnlocked) {
            climbTimer.reset();
        }
        updateServos();
    }

    private void updateServos() {
        if (isPtoUnlocked) {
            archi.setPosition(0.45);
            medes.setPosition(0.55);
        } else {
            archi.setPosition(0.07);
            medes.setPosition(1.0);
        }
    }

    public void lockPto() {
        isPtoUnlocked = false;
        updateServos();
    }

    public boolean isPtoUnlocked() {
        return isPtoUnlocked;
    }

    public void runAutoClimb(boolean isClimbing, Telemetry telemetry) {

        if (!isClimbing) {
            return;
        }

        if (!isPtoUnlocked) {
            isPtoUnlocked = true;
            climbTimer.reset();
            updateServos();
        }

        if (climbTimer.milliseconds() >= 200) {
            double currentRoll = imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);
            double flP = 0, frP = 0, blP = 0, brP = 0;

            if (currentRoll > ROLL_TOLERANCE) {
                flP = 0.85;  blP = -0.85;
                frP = 1.0; brP = -1.0;
                telemetry.addData("Climb Mode", "Leveling: Right Catching Up");
            } else if (currentRoll < -ROLL_TOLERANCE) {
                flP = 1.0; blP = -1.0;
                frP = 0.85;  brP = -0.85;
                telemetry.addData("Climb Mode", "Leveling: Left Catching Up");
            } else {
                flP = 1; frP = 1; blP = -1; brP = -1;
                telemetry.addData("Climb Mode", "Balanced Climbing");
            }

            fl.setPower(flP);
            bl.setPower(blP);
            fr.setPower(frP);
            br.setPower(brP);

            telemetry.addData("Hub Roll", "%.2f degrees", currentRoll);

        } else {
            fl.setPower(0);
            bl.setPower(0);
            fr.setPower(0);
            br.setPower(0);
            telemetry.addData("Climb Mode", "Waiting for PTO 300ms...");
        }
    }
}