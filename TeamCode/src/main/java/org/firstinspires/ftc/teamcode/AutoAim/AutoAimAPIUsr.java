package org.firstinspires.ftc.teamcode.AutoAim;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name="AutoAimAPIUsr", group="Production")
public class AutoAimAPIUsr extends LinearOpMode {

    private AutoAimSubsystem autoAim;

    final double TARGET_X_WORLD = 132.0;
    final double TARGET_Y_WORLD = 136.0;

    @Override
    public void runOpMode() {

        autoAim = new AutoAimSubsystem(hardwareMap, telemetry);
        Pose2D startPose = new Pose2D(DistanceUnit.INCH, 72, 72, AngleUnit.DEGREES, 0);
        autoAim.setInitialPose(startPose);

        telemetry.addLine("Ready to Start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            AutoAimSubsystem.TurretCommand aimCommand = autoAim.update(TARGET_X_WORLD, TARGET_Y_WORLD);

            if (aimCommand.hasTarget) {

                // turretPID        aimCommand.pidErrorDeg
                // shooterMotor     aimCommand.targetRpm
                // pitchServo       aimCommand.targetPitch

            } else {
                // 如果丢失目标，或者距离过近的默认处理
            }

            telemetry.update();
        }
    }
}