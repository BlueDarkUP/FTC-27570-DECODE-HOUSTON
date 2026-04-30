package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "🔵 Blue TeleOp", group = "Competition")
public class BlueTeleOp extends BaseTeleOp {

    @Override
    protected double getBaseTargetX() {
        return 135.0;
    }

    @Override
    protected double getBaseTargetY() {
        return 133.0;
    }

    @Override
    protected double getFarTargetX() {
        return 140.0;
    }

    @Override
    protected double getFarTargetY() {
        return 134.0;
    }

    @Override
    protected double getHeadingOffset() {
        return 90.0;
    }
}