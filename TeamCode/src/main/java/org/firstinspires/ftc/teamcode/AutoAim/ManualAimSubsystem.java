package org.firstinspires.ftc.teamcode.AutoAim;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config // 加上 Config 标签，使得这里的 public static 变量可以被 Dashboard 识别
public class ManualAimSubsystem {

    // 硬件声明
    private DcMotorEx Turret;
    private Servo LP;
    private Servo RP;

    // =======================================================
    // Dashboard 实时调参区 (修改这里的值会立刻在机器人上生效)
    // =======================================================

    // 云台 PIDF 参数 (保留了你的 kD，通过 Dashboard 觉得震动大再自己微调)
    public static double TURRET_kP = 0.003;
    public static double TURRET_kI = 0.0;
    public static double TURRET_kD = 0.0001;
    public static double TURRET_kF = 0.0;

    // 死区阈值 (单位: 度)。当误差小于这个值时，切断电机电流，解决高频震动！
    // 如果飞轮转的时候还是震，可以适当调大到 1.5 甚至 2.0
    public static double TURRET_DEADZONE_DEG = 1.0;

    // 云台最大输出限幅。限制电机的狂暴抽搐，0.6 表示最大 60% 功率
    public static double TURRET_MAX_POWER = 0.6;

    // =======================================================

    // 控制器与常量
    private PIDFController turretPIDF;
    private final double TICKS_PER_REV = 32798.0;

    private final double LP_UP = 1.0;
    private final double LP_DOWN = 0.4;
    private final double RP_UP = 0.0;
    private final double RP_DOWN = 0.5;

    // 允许的云台锁定误差角 (度)
    // 注意：这个参数仅用于返回给主程序判断 "是否锁定"，和上面那个断电的死区是两码事
    private final double AIM_LOCK_TOLERANCE_DEG = 1.5;

    /**
     * 封装传回主 TeleOp 的结果对象
     */
    public static class TurretCommand {
        public boolean hasTarget = false;
        public double targetRpm = 0.0;
        public double targetPitch = 0.0;
        public boolean isAimLocked = false;
        public boolean isUnwinding = false; // 是否触发了防绞线复位
    }

    public ManualAimSubsystem(HardwareMap hardwareMap) {
        // 初始化云台电机
        Turret = hardwareMap.get(DcMotorEx.class, "Turret");
        Turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // 初始化俯仰舵机
        LP = hardwareMap.get(Servo.class, "LP");
        RP = hardwareMap.get(Servo.class, "RP");

        // 初始化 PID 控制器
        turretPIDF = new PIDFController(TURRET_kP, TURRET_kI, TURRET_kD, TURRET_kF);

        // 默认俯仰居中
        setPitchServos(0.7);
    }

    /**
     * 同步控制左右俯仰舵机
     */
    private void setPitchServos(double targetPitch) {
        double clampedLP = Math.max(LP_DOWN, Math.min(LP_UP, targetPitch));
        double proportion = (clampedLP - LP_DOWN) / (LP_UP - LP_DOWN);
        double calculatedRP = RP_DOWN + proportion * (RP_UP - RP_DOWN);
        calculatedRP = Math.max(0.0, Math.min(1.0, calculatedRP));

        LP.setPosition(clampedLP);
        RP.setPosition(calculatedRP);
    }

    /**
     * 核心更新函数：传入从主程序中读取到的传感器数据进行自瞄控制
     */
    public TurretCommand update(
            double robotX, double robotY, double globalVx, double globalVy, double currentHeadingDeg,
            double targetX, double targetY,
            boolean isManualMode, double manualDist) {

        // 在每一帧中，将 Dashboard 的参数实时更新到 PID 控制器中
        turretPIDF.setPIDF(TURRET_kP, TURRET_kI, TURRET_kD, TURRET_kF);

        TurretCommand command = new TurretCommand();

        // 计算自动瞄准
        AimCalculator.AimResult aimResult;

        if (isManualMode) {
            double manualRpm = AimCalculator.interpolate(manualDist, 1);
            double manualPitch = AimCalculator.interpolate(manualDist, 2);
            aimResult = new AimCalculator.AimResult(manualDist, currentHeadingDeg, manualRpm, manualPitch, 0.0);
        } else {
            aimResult = AimCalculator.solveAim(
                    robotX, robotY, globalVx, globalVy, 0, 0, targetX, targetY
            );
        }

        if (aimResult != null) {
            command.hasTarget = true;
            command.targetRpm = aimResult.rpm;
            command.targetPitch = aimResult.pitch;

            // =============== 偏航角 (Yaw) 控制逻辑 ===============
            double currentTurretTicks = Turret.getCurrentPosition();
            double currentTurretRelAngle = (currentTurretTicks / TICKS_PER_REV) * 360.0;
            double currentTurretAbsAngle = currentHeadingDeg + currentTurretRelAngle;

            double error = aimResult.algYaw - currentTurretAbsAngle;

            // 将误差规范化到 [-180, 180] 之间找最短物理路径
            while (error > 180) error -= 360;
            while (error < -180) error += 360;

            double targetTurretRelAngle = currentTurretRelAngle + error;

            // 程序限位与优弧防绞线逻辑：逆时针最大175，顺时针最大-210
            if (targetTurretRelAngle > 175.0) {
                targetTurretRelAngle -= 360.0;
                command.isUnwinding = true;
            } else if (targetTurretRelAngle < -210.0) {
                targetTurretRelAngle += 360.0;
                command.isUnwinding = true;
            } else {
                command.isUnwinding = false;
            }

            // 判断云台是否已瞄准目标容差范围内
            command.isAimLocked = Math.abs(error) <= AIM_LOCK_TOLERANCE_DEG;

            // PID 运算获取原始输出
            double targetTurretTicks = (targetTurretRelAngle / 360.0) * TICKS_PER_REV;
            double turretPower = turretPIDF.calculate(currentTurretTicks, targetTurretTicks);

            // ================= 防震荡优化核心逻辑 =================
            // 1. 如果角度误差在死区阈值内，直接断开电机电源，消除因飞轮震动带来的 PID 抖动
            if (Math.abs(error) < TURRET_DEADZONE_DEG) {
                turretPower = 0.0;
                turretPIDF.reset(); // 清除积分累计，防止出死区瞬间爆冲
            }

            // 2. 软限幅，避免剧烈抽搐产生的巨大机械反冲力
            turretPower = Math.max(-TURRET_MAX_POWER, Math.min(TURRET_MAX_POWER, turretPower));
            // ======================================================

            Turret.setPower(turretPower);

            // 更新舵机
            setPitchServos(command.targetPitch);
        } else {
            Turret.setPower(0);
        }

        return command;
    }

    // 强制断电接口，供结束或急停时使用
    public void stop() {
        Turret.setPower(0);
    }
}