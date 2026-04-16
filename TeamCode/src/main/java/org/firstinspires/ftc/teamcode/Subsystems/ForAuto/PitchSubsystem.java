package org.firstinspires.ftc.teamcode.Subsystems.ForAuto;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PitchSubsystem {
    private Servo LP;
    private Servo RP;

    private final double LP_UP = 1.0;
    private final double LP_DOWN = 0.4;
    private final double RP_UP = 0.0;
    private final double RP_DOWN = 0.5;

    public PitchSubsystem(HardwareMap hardwareMap) {
        LP = hardwareMap.get(Servo.class, "LP");
        RP = hardwareMap.get(Servo.class, "RP");
    }

    public void setPitch(double targetPitch) {
        double clampedLP = Math.max(LP_DOWN, Math.min(LP_UP, targetPitch));
        double proportion = (clampedLP - LP_DOWN) / (LP_UP - LP_DOWN);
        double calculatedRP = RP_DOWN + proportion * (RP_UP - RP_DOWN);
        calculatedRP = Math.max(0.0, Math.min(1.0, calculatedRP));

        LP.setPosition(clampedLP);
        RP.setPosition(calculatedRP);
    }
}