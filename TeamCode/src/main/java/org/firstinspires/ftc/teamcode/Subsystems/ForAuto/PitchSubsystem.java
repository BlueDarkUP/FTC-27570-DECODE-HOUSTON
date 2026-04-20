package org.firstinspires.ftc.teamcode.Subsystems.ForAuto;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.GlobalConstants;

public class PitchSubsystem {
    private Servo LP;
    private Servo RP;

    public PitchSubsystem(HardwareMap hardwareMap) {
        LP = hardwareMap.get(Servo.class, "LP");
        RP = hardwareMap.get(Servo.class, "RP");
    }

    public void setPitch(double targetPitch) {
        double clampedLP = Math.max(GlobalConstants.PITCH_LP_DOWN, Math.min(GlobalConstants.PITCH_LP_UP, targetPitch));
        double proportion = (clampedLP - GlobalConstants.PITCH_LP_DOWN) / (GlobalConstants.PITCH_LP_UP - GlobalConstants.PITCH_LP_DOWN);

        double calculatedRP = GlobalConstants.PITCH_RP_DOWN + proportion * (GlobalConstants.PITCH_RP_UP - GlobalConstants.PITCH_RP_DOWN);
        calculatedRP = Math.max(0.0, Math.min(1.0, calculatedRP));

        LP.setPosition(clampedLP);
        RP.setPosition(calculatedRP);
    }
}