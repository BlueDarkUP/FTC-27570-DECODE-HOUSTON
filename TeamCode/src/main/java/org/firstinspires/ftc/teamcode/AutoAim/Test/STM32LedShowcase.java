/*
 * Copyright (c) 2024-2025
 *
 * This is the official showcase OpMode for the STM32 Smart LED Module (V3 Engine).
 * It demonstrates how to utilize the advanced rendering capabilities including:
 * - Basic Colors & Strobes
 * - Non-linear Breath / Gradients (ENTB)
 * - Sub-pixel Anti-aliased Waves (ENTW2)
 * - Smart Progress Bars & Lasers (ENTPB)
 */

package org.firstinspires.ftc.teamcode.AutoAim.Test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.vision.KIRIN.STM32LedModule;
import org.firstinspires.ftc.teamcode.vision.KIRIN.STM32LedModule.Color;
import org.firstinspires.ftc.teamcode.vision.KIRIN.STM32LedModule.Direction;
import org.firstinspires.ftc.teamcode.vision.KIRIN.STM32LedModule.FadeType;


@Autonomous(name = "🌟 STM32 V3 LED Showcase", group = "Demo")
public class STM32LedShowcase extends LinearOpMode {

    private STM32LedModule led;

    @Override
    public void runOpMode() {
        // 1. Initialize the hardware map
        // Ensure the name "STM32LedModule" matches your Robot Controller Configuration
        led = hardwareMap.get(STM32LedModule.class, "STM32LedModule");

        telemetry.addLine("STM32 Smart LED V3 Engine Showcase");
        telemetry.addLine("-----------------------------------");
        telemetry.addLine("Press START to begin the 40-second light show.");
        telemetry.update();

        // 2. Assign the ID. This stops the boot-up orange flashing and sets it to standby.
        // Once assigned, you can omit the 'A' in all subsequent calls!
        led.assignId('A');

        waitForStart();

        if (opModeIsActive()) {

            // ==========================================
            // Phase 1: The Basics (Solid, Wave, Strobe)
            // ==========================================
            printStep("Phase 1/5: The Basics", "Solid RED");
            // Set solid red at 100 brightness
            led.setSolidColor(Color.RED, 100);
            sleep(1500);

            printStep("Phase 1/5: The Basics", "Classic Dual Wave (Cyan/Magenta)");
            // Cyan and Magenta, 150% speed, 100 brightness
            led.setWaveMode(Color.CYAN, Color.MAGENTA, 150, 100);
            sleep(2500);

            printStep("Phase 1/5: The Basics", "Police Strobe (Red/Blue)");
            // Red and Blue alternating strobe, 120% speed
            led.setStrobeMode(Color.RED, Color.BLUE, 120, 100);
            sleep(2500);


            // ==========================================
            // Phase 2: Advanced Breath Engine (ENTB)
            // ==========================================
            printStep("Phase 2/5: Breath Engine", "Global Breath (Red -> Orange -> Yellow)");
            // isAllPixels = true: The entire strip breathes together
            // Easing = 1.5: Creates a non-linear, natural breathing curve
            led.setBreathMode(true, 100, 1.5, 100, Color.RED, Color.ORANGE, Color.YELLOW);
            sleep(4000);

            printStep("Phase 2/5: Breath Engine", "Cyberpunk Flowing Gradient");
            // isAllPixels = false: The colors flow across the strip while breathing
            led.setBreathMode(false, 150, 2.0, 100, Color.PURPLE, Color.CYAN);
            sleep(4000);


            // ==========================================
            // Phase 3: Ultimate Anti-Aliased Stripe (ENTW2)
            // ==========================================
            printStep("Phase 3/5: Sub-Pixel Engine", "Hard Edge Blocks (Traffic Light)");
            // Width = 3.0, Gap = 1.0, Anti-alias = false
            // Looks like distinct physical blocks moving right
            led.setAdvancedStripeWave(3.0, 1.0, Direction.RIGHT, false, 1.0, 100, 100, Color.RED, Color.YELLOW, Color.GREEN);
            sleep(3000);

            printStep("Phase 3/5: Sub-Pixel Engine", "Smooth Blended Aura (Anti-Aliased)");
            // Anti-alias = true, Easing = 2.0
            // The exact same colors now blend seamlessly across the LEDs
            led.setAdvancedStripeWave(3.0, 1.0, Direction.LEFT, true, 2.0, 80, 100, Color.RED, Color.YELLOW, Color.GREEN);
            sleep(4000);


            // ==========================================
            // Phase 4: Smart Progress Bar - Auto (ENTPB)
            // ==========================================
            printStep("Phase 4/5: Progress Bar", "Auto Laser Blast");
            led.turnOff();
            sleep(500);

            // Fires an automatic animation:
            // Fills Cyan over 1.0 seconds (accelerating A=2.0)
            // Holds for 0.2 seconds
            // Erases in the same direction (FB) over 0.5 seconds
            led.triggerAutoProgressBar(
                    Color.CYAN, Color.BLACK,
                    1.0, 2.0, FadeType.DIRECTIONAL_ERASE, 0.2, 0.5, Direction.RIGHT, 100
            );

            // The STM32 handles the entire animation autonomously!
            // We just need to wait for it to finish.
            sleep(2000);


            // ==========================================
            // Phase 5: Smart Progress Bar - Manual
            // ==========================================
            printStep("Phase 5/5: Accumulation", "Simulating Intake Loading...");
            led.turnOff();
            sleep(200);

            // Simulating picking up 3 game elements (e.g., rings or pixels)
            // Total grid size is 10 units. We add 3.34 units per element.
            for (int i = 1; i <= 3; i++) {
                telemetry.addData("Loaded Element", i + "/3");
                telemetry.update();

                // Beep gets higher pitch with each element
                led.beep(2000 + i * 500, 100);

                // Add 3.34 units of Green color over 0.3 seconds per element
                led.addManualProgress(Color.GREEN, 3.34, 0.3, 1.0, Direction.RIGHT, 100);

                sleep(1000); // Wait 1 second between pickups
            }
            sleep(1000);


            // ==========================================
            // Finale
            // ==========================================
            printStep("Showcase Complete!", "Turning off...");
            led.setSolidColor(Color.WHITE, 100);
            led.beep(2700, 500); // Final confirmation beep
            sleep(100);
            led.turnOff();
        }
    }

    /**
     * Helper method to print the current step to the Driver Station.
     */
    private void printStep(String phase, String action) {
        telemetry.clearAll();
        telemetry.addLine(">>> " + phase + " <<<");
        telemetry.addData("Current Effect", action);
        telemetry.update();
    }
}