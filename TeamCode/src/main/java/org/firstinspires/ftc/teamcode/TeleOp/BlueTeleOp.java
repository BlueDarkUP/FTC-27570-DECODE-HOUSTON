package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "🔵 Blue TeleOp", group = "Competition")
public class BlueTeleOp extends BaseTeleOp {

    @Override
    protected double getTargetX() {
        return 136.0;
    }
    @Override
    protected double getTargetY() {
        return 133.0;
    }
    @Override
    protected double getHeadingOffset() {
        return 90.0;
    }
}