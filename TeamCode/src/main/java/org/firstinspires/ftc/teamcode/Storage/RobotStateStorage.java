package org.firstinspires.ftc.teamcode.Storage;

public class RobotStateStorage {
    public static boolean isAutoDataValid = false;
    public static double turretAngleDeg = 0.0;
    public static double odoX = 72.0;
    public static double odoY = 72.0;
    public static double odoHeading = 0.0;

    public static void clear() {
        isAutoDataValid = false;
        turretAngleDeg = 0.0;
        odoX = 72.0;
        odoY = 72.0;
        odoHeading = 0.0;
    }
}