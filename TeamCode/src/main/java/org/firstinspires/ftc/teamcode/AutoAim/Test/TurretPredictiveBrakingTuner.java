package org.firstinspires.ftc.teamcode.AutoAim.Test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;

@Config
@TeleOp(name = "Turret PB Tuner", group = "Tuning")
public class TurretPredictiveBrakingTuner extends OpMode {

    private DcMotorEx Turret;
    private final double TICKS_PER_REV = 32798.0; //TODO: 云台绕一圈编码器读数

    public static double[] TEST_POWERS = {1.0, 0.9, 0.8, 0.7, 0.6, 0.5, 0.4, 0.3};

    public static double BRAKING_POWER = 0.3;

    public static int SPIN_TIME_MS = 3000;

    public static int REST_TIME_MS = 1000;

    public static double MAX_SAFE_ANGLE = 120.0;
    public static double BRAKE_BUFFER_ANGLE = 40.0;
    public static double VEL_FILTER_ALPHA = 0.9;

    private enum State {
        START_MOVE,
        WAIT_SPIN,
        APPLY_BRAKE,
        WAIT_BRAKE,
        REST,
        DONE
    }

    private State state = State.START_MOVE;
    private final ElapsedTime timer = new ElapsedTime();
    private int iteration = 0;

    private double filteredVel = 0.0;
    private double lastAngle = 0.0;
    private long lastTime = 0;
    private boolean isInitialized = false;

    private double startBrakeAngle = 0.0;
    private double measuredBrakeVel = 0.0;
    private int currentDirection = 1;

    private final List<double[]> collectedData = new ArrayList<>();

    private double finalKLinear = 0.0;
    private double finalKQuadratic = 0.0;

    @Override
    public void init() {
        Turret = hardwareMap.get(DcMotorEx.class, "Turret");
        Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // 这一步将当前位置设为绝对0度
        Turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Ready to tune Turret Predictive Braking.");
        telemetry.addLine("WARNING: Keep clear of the Turret. It will spin rapidly back and forth.");
        telemetry.addData("Safe Angle Limit", MAX_SAFE_ANGLE + " deg");
        telemetry.addData("Brake Trigger Angle", (MAX_SAFE_ANGLE - BRAKE_BUFFER_ANGLE) + " deg");
        telemetry.update();
    }

    @Override
    public void start() {
        timer.reset();
        lastTime = System.nanoTime();
    }

    @Override
    public void loop() {
        // --- 1. 读取并计算当前的真实速度 (带低通滤波) ---
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1e9;
        lastTime = currentTime;

        double currentTicks = Turret.getCurrentPosition();
        double currentAngle = (currentTicks / TICKS_PER_REV) * 360.0;

        if (!isInitialized) {
            lastAngle = currentAngle;
            filteredVel = 0.0;
            isInitialized = true;
        }

        double rawVel = 0.0;
        if (dt > 0.0001) {
            rawVel = (currentAngle - lastAngle) / dt;
        }
        lastAngle = currentAngle;

        filteredVel = (VEL_FILTER_ALPHA * rawVel) + ((1.0 - VEL_FILTER_ALPHA) * filteredVel);

        switch (state) {
            case START_MOVE:
                if (iteration >= TEST_POWERS.length * 2) {
                    state = State.DONE;
                    break;
                }

                int powerIndex = iteration / 2;
                currentDirection = (iteration % 2 == 0) ? 1 : -1;
                double power = TEST_POWERS[powerIndex] * currentDirection;

                Turret.setPower(power);
                timer.reset();
                state = State.WAIT_SPIN;
                break;

            case WAIT_SPIN:
                boolean timeUp = timer.milliseconds() >= SPIN_TIME_MS;

                double limitThreshold = MAX_SAFE_ANGLE - BRAKE_BUFFER_ANGLE;
                boolean hitSafetyLimit = false;
                if (currentDirection == 1 && currentAngle >= limitThreshold) {
                    hitSafetyLimit = true;
                } else if (currentDirection == -1 && currentAngle <= -limitThreshold) {
                    hitSafetyLimit = true;
                }

                if (timeUp || hitSafetyLimit) {
                    measuredBrakeVel = filteredVel;
                    startBrakeAngle = currentAngle;

                    Turret.setPower(-currentDirection * BRAKING_POWER);
                    timer.reset();
                    state = State.WAIT_BRAKE;
                }
                break;

            case WAIT_BRAKE:
                if (Math.signum(filteredVel) != currentDirection || Math.abs(filteredVel) < 2.0) {
                    Turret.setPower(0);

                    double brakingDistance = Math.abs(currentAngle - startBrakeAngle);
                    double absVel = Math.abs(measuredBrakeVel);

                    collectedData.add(new double[]{absVel, brakingDistance});

                    iteration++;
                    timer.reset();
                    state = State.REST;
                }
                break;

            case REST:
                if (timer.milliseconds() >= REST_TIME_MS) {
                    state = State.START_MOVE;
                }
                break;

            case DONE:
                Turret.setPower(0);

                if (finalKLinear == 0.0 && finalKQuadratic == 0.0 && !collectedData.isEmpty()) {
                    calculateCoefficients();
                }

                telemetry.addLine("=== TUNING COMPLETE ===");
                telemetry.addData("TURRET_kLinearBraking", String.format("%.6f", finalKLinear));
                telemetry.addData("TURRET_kQuadraticFriction", String.format("%.6f", finalKQuadratic));
                telemetry.addLine("\nCopy these values to your AutoAimSubsystem!");
                break;
        }

        if (state != State.DONE) {
            telemetry.addData("State", state.toString());
            telemetry.addData("Iteration", iteration + " / " + (TEST_POWERS.length * 2));
            telemetry.addData("Current Angle (deg)", currentAngle);
            telemetry.addData("Current Vel (deg/s)", filteredVel);
            telemetry.addData("Test Power", (iteration < TEST_POWERS.length * 2) ? (TEST_POWERS[iteration / 2] * currentDirection) : 0);

            if (Math.abs(currentAngle) > MAX_SAFE_ANGLE) {
                telemetry.addLine("!!! DANGER: PAST SAFE ANGLE !!!");
            }
        }

        telemetry.update();
    }

    private void calculateCoefficients() {
        double sumX = 0;
        double sumY = 0;
        double sumXY = 0;
        double sumXX = 0;
        int n = 0;

        for (double[] point : collectedData) {
            double v = point[0];
            double d = point[1];

            if (v < 10.0) continue;

            double x = v;
            double y = d / v;

            sumX += x;
            sumY += y;
            sumXY += x * y;
            sumXX += x * x;
            n++;
        }

        if (n > 1) {
            double denominator = (n * sumXX) - (sumX * sumX);
            finalKQuadratic = ((n * sumXY) - (sumX * sumY)) / denominator;
            finalKLinear = ((sumY * sumXX) - (sumX * sumXY)) / denominator;

            if (finalKQuadratic < 0) finalKQuadratic = 0;
            if (finalKLinear < 0) finalKLinear = 0;
        }
    }
}