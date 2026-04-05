/*   MIT License
 *   Copyright (c) 2024-2025
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:
 *
 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.
 *
 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Driver.KIRIN;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.RobotLog;

import java.nio.charset.StandardCharsets;
import java.util.Locale;

/**
 * <h1>STM32 Smart LED & Buzzer Module Driver (V3 Firmware)</h1>
 * <p>A high-performance, non-blocking I2C driver for the STM32-based daisy-chainable LED module.</p>
 * <p>Features built-in rendering engines for breathing, anti-aliased waves, and smart progress bars.</p>
 */
@I2cDeviceType
@DeviceProperties(
        name = "STM32 Smart LED Module V3",
        xmlTag = "STM32LedModule",
        description = "Advanced I2C LED & Buzzer Driver (V3 Engine)"
)
public class STM32LedModule extends I2cDeviceSynchDevice<I2cDeviceSynch> {

    public static final I2cAddr DEFAULT_ADDRESS = I2cAddr.create7bit(0x60);
    private static final String TAG = "STM32_LED";

    // Default ID for single-module setups
    public char defaultId = 'A';

    // ==========================================
    // Public Enums & Constants
    // ==========================================

    /** Standard Hex Colors for Convenience */
    public static class Color {
        public static final int RED     = 0xFF0000;
        public static final int GREEN   = 0x00FF00;
        public static final int BLUE    = 0x0000FF;
        public static final int WHITE   = 0xFFFFFF;
        public static final int BLACK   = 0x000000; // OFF
        public static final int YELLOW  = 0xFFFF00;
        public static final int CYAN    = 0x00FFFF;
        public static final int MAGENTA = 0xFF00FF;
        public static final int ORANGE  = 0xFFA500;
        public static final int PURPLE  = 0x800080;
        public static final int GOLD    = 0xFFD700;
        public static final int PINK    = 0xFFC0CB;
    }

    /** Flow Direction for Animations */
    public enum Direction {
        RIGHT("R"), LEFT("L");
        public final String code;
        Direction(String code) { this.code = code; }
    }

    /** Erase/Fade behavior for Progress Bar */
    public enum FadeType {
        GLOBAL_FADE("F"),          // Whole bar fades out at once
        DIRECTIONAL_ERASE("FB"),   // Erases in the same direction it filled
        NONE("");                  // Does not fade automatically
        public final String code;
        FadeType(String code) { this.code = code; }
    }

    // ==========================================
    // Initialization
    // ==========================================

    public STM32LedModule(I2cDeviceSynch deviceClient, boolean deviceClientIsOwned) {
        super(deviceClient, deviceClientIsOwned);
        this.deviceClient.setI2cAddress(DEFAULT_ADDRESS);
        super.registerArmingStateCallback(false);
        this.deviceClient.engage();
    }

    @Override public Manufacturer getManufacturer() { return Manufacturer.Other; }
    @Override public String getDeviceName() { return "STM32 Smart LED Module V3"; }
    @Override protected synchronized boolean doInitialize() { return true; }

    // ==========================================
    // Internal Core Methods
    // ==========================================

    private void sendCommand(String command) {
        String fullCommand = command + "&";
        byte[] payload = fullCommand.getBytes(StandardCharsets.US_ASCII);
        try {
            this.deviceClient.write(payload);
        } catch (Exception e) {
            RobotLog.ee(TAG, "I2C Write Failed: " + fullCommand);
        }
    }

    private String colorToHex(int color) {
        return String.format(Locale.US, "%06X", (color & 0xFFFFFF));
    }

    private int clamp(int val, int min, int max) {
        return Math.max(min, Math.min(max, val));
    }

    // ==========================================
    // Basic API (V1 / V2 Compatible)
    // ==========================================

    /**
     * Initialize and assign an ID to the module. Stops the boot-up orange flashing.
     * @param id The ID to assign (e.g., 'A').
     */
    public void assignId(char id) {
        this.defaultId = id;
        sendCommand("ASSIGN " + id);
    }
    public void assignId() { assignId(defaultId); }

    /** Solid Color (ENT) */
    public void setSolidColor(char id, int color, int brightness) {
        sendCommand(String.format(Locale.US, "%c ENT %s %d", id, colorToHex(color), clamp(brightness, 0, 255)));
    }
    public void setSolidColor(int color, int brightness) { setSolidColor(defaultId, color, brightness); }

    /** Classic Dual-Color Wave (ENTW) */
    public void setWaveMode(char id, int c1, int c2, int speedPercent, int brightness) {
        sendCommand(String.format(Locale.US, "%c ENTW %s %s %d%% %d", id, colorToHex(c1), colorToHex(c2), speedPercent, clamp(brightness, 0, 255)));
    }
    public void setWaveMode(int c1, int c2, int speedPercent, int brightness) { setWaveMode(defaultId, c1, c2, speedPercent, brightness); }

    /** Police Strobe (ENTST) */
    public void setStrobeMode(char id, int c1, int c2, int speedPercent, int brightness) {
        sendCommand(String.format(Locale.US, "%c ENTST %s %s %d%% %d", id, colorToHex(c1), colorToHex(c2), speedPercent, clamp(brightness, 0, 255)));
    }
    public void setStrobeMode(int c1, int c2, int speedPercent, int brightness) { setStrobeMode(defaultId, c1, c2, speedPercent, brightness); }

    /** Set Individual LED (ENAT) */
    public void setIndividualLed(char id, int ledIndex, int color, int brightness) {
        int actualIndex = ledIndex == 0 ? 1 : ledIndex; // Ensure 1-based index for C backend
        sendCommand(String.format(Locale.US, "%c ENAT L%d %s %d", id, actualIndex, colorToHex(color), clamp(brightness, 0, 255)));
    }
    public void setIndividualLed(int ledIndex, int color, int brightness) { setIndividualLed(defaultId, ledIndex, color, brightness); }

    /** Trigger Buzzer (BENT) */
    public void beep(char id, int freqHz, int durationMs) {
        sendCommand(String.format(Locale.US, "%c BENT %d %d", id, freqHz, durationMs));
    }
    public void beep(int freqHz, int durationMs) { beep(defaultId, freqHz, durationMs); }

    /** Turn off all LEDs on the module (ENF) */
    public void turnOff(char id) { sendCommand(id + " ENF"); }
    public void turnOff() { turnOff(defaultId); }


    // ==========================================
    // Advanced V3 Engine API
    // ==========================================

    /**
     * Mode 5: Advanced Breath / Gradient (ENTB)
     * @param isAllPixels True for global breath, False for flowing gradient.
     * @param speedPercent Animation speed (100 is default).
     * @param easingA Non-linear easing factor (1.0 is linear, 2.0 is sharper).
     * @param brightness Max brightness (0-255).
     * @param colors Varargs array of up to 4 colors.
     */
    public void setBreathMode(char id, boolean isAllPixels, int speedPercent, double easingA, int brightness, int... colors) {
        StringBuilder sb = new StringBuilder();
        sb.append(id).append(" ENTB ");
        if (isAllPixels) sb.append("ALL ");

        for (int i = 0; i < Math.min(colors.length, 4); i++) {
            sb.append(colorToHex(colors[i])).append(" ");
        }
        sb.append(speedPercent).append("% ")
                .append(String.format(Locale.US, "a%.2f ", easingA))
                .append(clamp(brightness, 0, 255));
        sendCommand(sb.toString());
    }
    public void setBreathMode(boolean isAllPixels, int speedPercent, double easingA, int brightness, int... colors) {
        setBreathMode(defaultId, isAllPixels, speedPercent, easingA, brightness, colors);
    }

    /**
     * Mode 6: Advanced Stripe Wave with Sub-pixel Anti-Aliasing (ENTW2)
     * @param width Width of each color band.
     * @param gap Gap between color bands.
     * @param dir Direction of flow.
     * @param antiAlias Enable sub-pixel cross-fade smoothing.
     * @param easingA Smoothing curve factor (effective only if antiAlias is true).
     * @param speedPercent Flow speed.
     * @param brightness Max brightness.
     * @param colors Varargs array of up to 10 colors.
     */
    public void setAdvancedStripeWave(char id, double width, double gap, Direction dir, boolean antiAlias, double easingA, int speedPercent, int brightness, int... colors) {
        StringBuilder sb = new StringBuilder();
        sb.append(id).append(" ENTW2 ");
        for (int i = 0; i < Math.min(colors.length, 10); i++) {
            sb.append(colorToHex(colors[i])).append(" ");
        }
        sb.append(speedPercent).append("% ")
                .append(dir.code).append(" ")
                .append(String.format(Locale.US, "w%.2f ", width))
                .append(String.format(Locale.US, "s%.2f ", gap));

        if (antiAlias) {
            sb.append(String.format(Locale.US, "A%.2f ", easingA));
        }
        sb.append(clamp(brightness, 0, 255));
        sendCommand(sb.toString());
    }
    public void setAdvancedStripeWave(double width, double gap, Direction dir, boolean antiAlias, double easingA, int speedPercent, int brightness, int... colors) {
        setAdvancedStripeWave(defaultId, width, gap, dir, antiAlias, easingA, speedPercent, brightness, colors);
    }

    /**
     * Mode 7A: Auto Smart Progress Bar (ENTPB)
     * Fires a one-shot filling animation (e.g. Laser blast, charging).
     *
     * @param c1 Fill color
     * @param c2 Background color
     * @param fillTimeSeconds Time to fill the bar.
     * @param accelA Acceleration curve (1.0 is linear).
     * @param fadeType How the bar disappears after filling.
     * @param holdTimeSeconds How long to hold at 100% full.
     * @param fadeTimeSeconds How long the fade/erase animation takes.
     * @param dir Fill direction.
     * @param brightness Max brightness.
     */
    public void triggerAutoProgressBar(char id, int c1, int c2, double fillTimeSeconds, double accelA, FadeType fadeType, double holdTimeSeconds, double fadeTimeSeconds, Direction dir, int brightness) {
        String fadeStr = fadeType == FadeType.NONE ? "" : String.format(Locale.US, "%s%.2f-%.2f ", fadeType.code, holdTimeSeconds, fadeTimeSeconds);
        String cmd = String.format(Locale.US, "%c ENTPB %s %s T%.2f A%.2f %s%s %d",
                id, colorToHex(c1), colorToHex(c2), fillTimeSeconds, accelA, fadeStr, dir.code, clamp(brightness, 0, 255));
        sendCommand(cmd);
    }
    public void triggerAutoProgressBar(int c1, int c2, double fillTimeSeconds, double accelA, FadeType fadeType, double holdTimeSeconds, double fadeTimeSeconds, Direction dir, int brightness) {
        triggerAutoProgressBar(defaultId, c1, c2, fillTimeSeconds, accelA, fadeType, holdTimeSeconds, fadeTimeSeconds, dir, brightness);
    }

    /**
     * Mode 7B: Manual Progress Bar Accumulation (ENTPB M)
     * Adds units to the bar with a smooth animation.
     *
     * @param color Fill color
     * @param addUnits Number of grid units to add (Total is 10 units).
     * @param fillTimeSeconds Animation time for this addition.
     * @param accelA Acceleration curve.
     * @param dir Direction of accumulation.
     */
    public void addManualProgress(char id, int color, double addUnits, double fillTimeSeconds, double accelA, Direction dir, int brightness) {
        String cmd = String.format(Locale.US, "%c ENTPB M %s U%.2f T%.2f A%.2f %s %d",
                id, colorToHex(color), addUnits, fillTimeSeconds, accelA, dir.code, clamp(brightness, 0, 255));
        sendCommand(cmd);
    }
    public void addManualProgress(int color, double addUnits, double fillTimeSeconds, double accelA, Direction dir, int brightness) {
        addManualProgress(defaultId, color, addUnits, fillTimeSeconds, accelA, dir, brightness);
    }
}