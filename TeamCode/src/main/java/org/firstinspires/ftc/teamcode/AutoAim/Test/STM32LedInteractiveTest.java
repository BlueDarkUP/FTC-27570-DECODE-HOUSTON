/*
 * Copyright (c) 2024-2025
 *
 * This is the interactive TeleOp showcase for the STM32 Smart LED Module (V3 Engine).
 * It supports controlling DAISY-CHAINED modules (Module A and Module B) using a Gamepad.
 */

package org.firstinspires.ftc.teamcode.AutoAim.Test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.KIRIN.STM32LedModule;
import org.firstinspires.ftc.teamcode.vision.KIRIN.STM32LedModule.Color;
import org.firstinspires.ftc.teamcode.vision.KIRIN.STM32LedModule.Direction;
import org.firstinspires.ftc.teamcode.vision.KIRIN.STM32LedModule.FadeType;

@TeleOp(name = "🎮 STM32 V3 LED 手柄双灯联调", group = "Demo")
public class STM32LedInteractiveTest extends LinearOpMode {

    private STM32LedModule led;

    // 枚举：控制目标 (灯 A, 灯 B, 或者同时控制)
    private enum Target {
        MODULE_A('A'),
        MODULE_B('B'),
        BOTH('*');

        public final char id;
        Target(char id) { this.id = id; }
    }

    // 枚举：当前演示的功能菜单
    private enum DemoMode {
        BASIC_COLORS("1. 基础颜色 (常亮)"),
        CLASSIC_ANIM("2. 经典动画 (波浪/爆闪)"),
        ADV_BREATH("3. 高级呼吸引擎 (ENTB)"),
        ADV_STRIPE("4. 无锯齿流动光带 (ENTW2)"),
        AUTO_PROGRESS("5. 自动进度条/激光 (ENTPB)"),
        MANUAL_PROGRESS("6. 手动进度积累 (ENTPB M)"),
        BUZZER("7. 蜂鸣器测试 (BENT)");

        public final String title;
        DemoMode(String title) { this.title = title; }
    }

    private Target currentTarget = Target.BOTH;
    private DemoMode currentMode = DemoMode.BASIC_COLORS;

    // 简易按键防抖工具类
    private static class Button {
        private boolean lastState = false;
        public boolean onPressed(boolean currentState) {
            boolean pressed = currentState && !lastState;
            lastState = currentState;
            return pressed;
        }
    }

    @Override
    public void runOpMode() {
        // 1. 初始化硬件
        led = hardwareMap.get(STM32LedModule.class, "STM32LedModule");

        // 实例化防抖按键
        Button btnUp = new Button();
        Button btnDown = new Button();
        Button btnLeft = new Button();
        Button btnRight = new Button();
        Button btnA = new Button();
        Button btnB = new Button();
        Button btnX = new Button();
        Button btnY = new Button();

        telemetry.addLine("STM32 LED 双灯互动测试已就绪");
        telemetry.addLine("请按下 START 开始...");
        telemetry.update();

        // 2. 分配 ID
        // 向级联的第一个模块发送 'A'，固件会自动让下一个模块变成 'B'
        led.assignId('A');

        waitForStart();

        while (opModeIsActive()) {

            // ================= 控制逻辑：目标切换 =================
            if (btnRight.onPressed(gamepad1.dpad_right)) {
                if (currentTarget == Target.MODULE_A) currentTarget = Target.MODULE_B;
                else if (currentTarget == Target.MODULE_B) currentTarget = Target.BOTH;
                else currentTarget = Target.MODULE_A;
            }
            if (btnLeft.onPressed(gamepad1.dpad_left)) {
                if (currentTarget == Target.BOTH) currentTarget = Target.MODULE_B;
                else if (currentTarget == Target.MODULE_B) currentTarget = Target.MODULE_A;
                else currentTarget = Target.BOTH;
            }

            // ================= 控制逻辑：模式切换 =================
            if (btnDown.onPressed(gamepad1.dpad_down)) {
                int nextIndex = (currentMode.ordinal() + 1) % DemoMode.values().length;
                currentMode = DemoMode.values()[nextIndex];
            }
            if (btnUp.onPressed(gamepad1.dpad_up)) {
                int prevIndex = (currentMode.ordinal() - 1 + DemoMode.values().length) % DemoMode.values().length;
                currentMode = DemoMode.values()[prevIndex];
            }

            // 全局关灯按键：无论在什么模式下，按下 Y 都会关闭当前选中的灯
            if (btnY.onPressed(gamepad1.y)) {
                executeOnTarget(id -> led.turnOff(id));
            }

            // 获取按键状态 (用于触发特效)
            boolean pressA = btnA.onPressed(gamepad1.a);
            boolean pressB = btnB.onPressed(gamepad1.b);
            boolean pressX = btnX.onPressed(gamepad1.x);

            // ================= 控制逻辑：特效触发 =================
            switch (currentMode) {
                case BASIC_COLORS:
                    if (pressA) executeOnTarget(id -> led.setSolidColor(id, Color.RED, 100));
                    if (pressB) executeOnTarget(id -> led.setSolidColor(id, Color.GREEN, 100));
                    if (pressX) executeOnTarget(id -> led.setSolidColor(id, Color.BLUE, 100));
                    break;

                case CLASSIC_ANIM:
                    if (pressA) executeOnTarget(id -> led.setWaveMode(id, Color.CYAN, Color.MAGENTA, 150, 100));
                    if (pressB) executeOnTarget(id -> led.setStrobeMode(id, Color.RED, Color.BLUE, 120, 100));
                    if (pressX) executeOnTarget(id -> led.setWaveMode(id, Color.YELLOW, Color.ORANGE, 100, 100));
                    break;

                case ADV_BREATH:
                    // A: 全局同色呼吸 (红->橙->黄)
                    if (pressA) executeOnTarget(id -> led.setBreathMode(id, true, 100, 1.5, 100, Color.RED, Color.ORANGE, Color.YELLOW));
                    // B: 赛博朋克流动呼吸 (紫->青)
                    if (pressB) executeOnTarget(id -> led.setBreathMode(id, false, 150, 2.0, 100, Color.PURPLE, Color.CYAN));
                    // X: 柔和单色呼吸 (白)
                    if (pressX) executeOnTarget(id -> led.setBreathMode(id, true, 80, 1.0, 80, Color.WHITE, Color.BLACK));
                    break;

                case ADV_STRIPE:
                    // A: 硬边缘红绿灯方块移动 (无锯齿平滑关闭)
                    if (pressA) executeOnTarget(id -> led.setAdvancedStripeWave(id, 3.0, 1.0, Direction.RIGHT, false, 1.0, 100, 100, Color.RED, Color.YELLOW, Color.GREEN));
                    // B: 柔和极光平滑融合 (开启亚像素级抗锯齿，曲线2.0)
                    if (pressB) executeOnTarget(id -> led.setAdvancedStripeWave(id, 4.0, 0.0, Direction.LEFT, true, 2.0, 80, 100, Color.CYAN, Color.BLUE, Color.PURPLE));
                    break;

                case AUTO_PROGRESS:
                    // A: 向右发射自动激光 (青色填满，自动同向擦除)
                    if (pressA) executeOnTarget(id -> led.triggerAutoProgressBar(id, Color.CYAN, Color.BLACK, 0.8, 2.0, FadeType.DIRECTIONAL_ERASE, 0.2, 0.4, Direction.RIGHT, 100));
                    // B: 能量护盾闪烁 (金色填满全亮，然后整体淡出消失)
                    if (pressB) executeOnTarget(id -> led.triggerAutoProgressBar(id, Color.GOLD, Color.BLACK, 0.5, 1.0, FadeType.GLOBAL_FADE, 0.5, 1.0, Direction.LEFT, 100));
                    break;

                case MANUAL_PROGRESS:
                    // A: 累加 2.0 个网格的绿色 (模拟吸入一个游戏元素)
                    if (pressA) executeOnTarget(id -> {
                        led.addManualProgress(id, Color.GREEN, 2.0, 0.3, 1.0, Direction.RIGHT, 100);
                        led.beep(id, 2000, 100);
                    });
                    // B: 累加 3.4 个网格的红色
                    if (pressB) executeOnTarget(id -> {
                        led.addManualProgress(id, Color.RED, 3.4, 0.4, 1.0, Direction.RIGHT, 100);
                        led.beep(id, 2500, 100);
                    });
                    // X: 一键清空 (由于 Y 键是全局关灯，这里 X 键也可以当清空用)
                    if (pressX) executeOnTarget(id -> led.turnOff(id));
                    break;

                case BUZZER:
                    // A: 低音滴
                    if (pressA) executeOnTarget(id -> led.beep(id, 1000, 200));
                    // B: 确认高音
                    if (pressB) executeOnTarget(id -> led.beep(id, 3000, 150));
                    // X: 极短促报错音
                    if (pressX) executeOnTarget(id -> led.beep(id, 500, 500));
                    break;
            }

            // ================= 屏幕显示 (UI) =================
            telemetry.addLine("=== 🌟 STM32 双灯交互控制台 ===");
            telemetry.addData("当前控制目标", "[<] %s [>]",
                    currentTarget == Target.BOTH ? "双灯 (A+B)" : (currentTarget == Target.MODULE_A ? "仅灯 A" : "仅灯 B"));

            telemetry.addData("当前功能菜单", "[↑] %s [↓]", currentMode.title);
            telemetry.addLine("--------------------------------");

            // 根据不同模式显示操作提示
            switch (currentMode) {
                case BASIC_COLORS:
                    telemetry.addLine("[A] 常亮红  [B] 常亮绿  [X] 常亮蓝");
                    break;
                case CLASSIC_ANIM:
                    telemetry.addLine("[A] 青紫波浪  [B] 警灯爆闪  [X] 黄橙波浪");
                    break;
                case ADV_BREATH:
                    telemetry.addLine("[A] 全局红黄呼吸  [B] 紫青流动呼吸  [X] 白色柔和呼吸");
                    break;
                case ADV_STRIPE:
                    telemetry.addLine("[A] 硬边缘红绿灯  [B] 柔和平滑极光(抗锯齿)");
                    break;
                case AUTO_PROGRESS:
                    telemetry.addLine("[A] 发射青色激光[B] 充能护盾(全局淡出)");
                    break;
                case MANUAL_PROGRESS:
                    telemetry.addLine("[A] +2格进度(绿)  [B] +3.4格进度(红)");
                    telemetry.addLine("[X] 清空进度");
                    break;
                case BUZZER:
                    telemetry.addLine("[A] 低频长音  [B] 高频短音  [X] 报错音");
                    break;
            }

            telemetry.addLine("\n全局控制:");
            telemetry.addLine("[Y] 一键关闭选中的灯光");
            telemetry.update();
        }
    }

    /**
     * 函数式接口回调辅助方法：根据当前的 target 状态，自动分发指令到 A灯、B灯或两者。
     */
    private void executeOnTarget(LedAction action) {
        if (currentTarget == Target.MODULE_A || currentTarget == Target.BOTH) {
            action.execute('A');
        }
        if (currentTarget == Target.MODULE_B || currentTarget == Target.BOTH) {
            action.execute('B');
        }
    }

    private interface LedAction {
        void execute(char id);
    }
}