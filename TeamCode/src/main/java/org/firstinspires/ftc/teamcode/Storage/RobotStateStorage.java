package org.firstinspires.ftc.teamcode.Storage;

public class RobotStateStorage {
    public static boolean isAutoDataValid = false;
    public static double turretAngleDeg = 0.0;
    public static void clear() {
        isAutoDataValid = false;
        turretAngleDeg = 0.0;
    }
}