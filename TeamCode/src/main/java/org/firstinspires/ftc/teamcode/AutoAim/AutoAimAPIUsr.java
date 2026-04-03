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

        telemetry.addLine("Initializing Hardware, Please wait...");
        telemetry.update();

        autoAim = new AutoAimSubsystem(hardwareMap, telemetry);

        sleep(500);

        Pose2D startPose = new Pose2D(DistanceUnit.INCH, 72, 72, AngleUnit.DEGREES, 0);
        autoAim.setInitialPose(startPose);

        while (opModeInInit()) {
            autoAim.update(TARGET_X_WORLD, TARGET_Y_WORLD);
            telemetry.addLine("Ready to Start - Odometry & Turret are Active!");
            telemetry.update();
        }

        waitForStart();

        while (opModeIsActive()) {

            // 核心调用：这一行代码搞定了定位、预测、防撞、弹道解算、PID运算以及电机输出！
            AutoAimSubsystem.TurretCommand command = autoAim.update(TARGET_X_WORLD, TARGET_Y_WORLD);

            if (command.hasTarget) {
                // shooterMotor.setVelocity(command.targetRpm);
                // pitchServo.setPosition(command.targetPitch);
            } else {
                // shooterMotor.setVelocity(0);
            }

            // 更新手机/Driver Hub上的控制台输出
            telemetry.update();
        }
    }
}