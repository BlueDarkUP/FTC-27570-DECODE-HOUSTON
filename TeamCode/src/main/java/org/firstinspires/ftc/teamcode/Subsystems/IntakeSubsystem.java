package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class IntakeSubsystem {

    // 硬件
    private DcMotorEx motorIntake;

    // 堵转保护常量
    private final double STALL_CURRENT_AMPS = 2.45;
    private final double STALL_COOLDOWN_SEC = 0.5;
    private final double STALL_TIME_THRESHOLD_SEC = 0.3;

    // 状态变量
    private double intakeBrakeReleaseTime = 0.0;
    private double stallStartTime = 0.0;
    private boolean isStalling = false;
    private double unwindReverseEndTime = 0.0;
    private boolean wasUnwinding = false;

    private ElapsedTime runtime = new ElapsedTime();

    // 状态反馈给 Telemetry
    private String systemStatusMessage = "初始化中...";

    public IntakeSubsystem(HardwareMap hardwareMap) {
        motorIntake = hardwareMap.get(DcMotorEx.class, "Intake");

        // 配置电机
        motorIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        motorIntake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        runtime.reset();
    }

    /**
     * 重置内部计时器，建议在 main loop 开始前调用
     */
    public void start() {
        runtime.reset();
    }

    /**
     * Intake 核心逻辑更新函数
     *
     * @param isShootingMode  当前是否处于发射模式
     * @param hasTarget       自瞄系统是否有目标
     * @param isUnwinding     是否需要云台复位/退弹
     * @param isAimLocked     云台是否已锁定目标
     * @param isFlywheelReady 飞轮转速是否已达标就绪
     */
    public void update(boolean isShootingMode, boolean hasTarget, boolean isUnwinding, boolean isAimLocked, boolean isFlywheelReady) {
        double currentTime = runtime.seconds();
        double intakeCurrent = motorIntake.getCurrent(CurrentUnit.AMPS);

        if (isShootingMode) {
            isStalling = false; // 射击模式下不判定普通收件的堵转

            // 云台退弹复位逻辑
            if (hasTarget && isUnwinding) {
                if (!wasUnwinding) {
                    unwindReverseEndTime = currentTime + 0.35;
                }

                if (currentTime < unwindReverseEndTime) {
                    motorIntake.setPower(-0.1);
                    systemStatusMessage = "⚠️ 云台复位中: Intake 反转退弹!";
                } else {
                    motorIntake.setPower(0.0);
                    systemStatusMessage = "⚠️ 云台复位中: Intake 刹车等待";
                }
                intakeBrakeReleaseTime = 0.0;
            } else {
                // 动态射击推弹逻辑
                if (hasTarget && isFlywheelReady && isAimLocked) {
                    motorIntake.setPower(1.0);
                    systemStatusMessage = "⚡ 动态射击中 (FIRE ON THE MOVE)!";
                } else {
                    motorIntake.setPower(0.0);
                    if (!hasTarget) {
                        systemStatusMessage = "等待目标 (或处于刹车保护)...";
                    } else if (!isFlywheelReady) {
                        systemStatusMessage = "飞轮蓄力调速中 (等待达到满转)...";
                    } else if (!isAimLocked) {
                        systemStatusMessage = "云台动态追瞄中 (等待角度容差锁定)...";
                    }
                }
            }
        } else {
            // 怠速收件及堵转保护逻辑
            if (intakeCurrent >= STALL_CURRENT_AMPS) {
                if (!isStalling) {
                    isStalling = true;
                    stallStartTime = currentTime;
                } else if (currentTime - stallStartTime >= STALL_TIME_THRESHOLD_SEC) {
                    intakeBrakeReleaseTime = currentTime + STALL_COOLDOWN_SEC;
                }
            } else {
                isStalling = false;
            }

            if (currentTime < intakeBrakeReleaseTime) {
                motorIntake.setPower(0.0);
                systemStatusMessage = "⚠️ INTAKE堵转保护触发！(刹车冷却中)";
                isStalling = false;
            } else {
                motorIntake.setPower(0.8);
                systemStatusMessage = "怠速中 (Intake 常转收件)";
            }
        }

        // 记录本周期的退弹状态供下周期判断边缘触发
        wasUnwinding = isUnwinding;
    }

    /**
     * 获取 Intake 系统的当前状态，用于主程序的 Telemetry 打印
     */
    public String getStatusMessage() {
        return systemStatusMessage;
    }
}