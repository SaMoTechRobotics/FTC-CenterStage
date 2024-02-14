package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.auto.AutoColor;
import org.firstinspires.ftc.teamcode.util.auto.AutoSide;

@TeleOp(name = "Auto: Red Near", group = "Autonomous")
public class AutoRedNear extends AutoBase {
    @Override
    protected AutoSide getSide() {
        return AutoSide.NEAR;
    }

    @Override
    protected AutoColor getColor() {
        return AutoColor.RED;
    }
}
