package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.auto.AutoColor;
import org.firstinspires.ftc.teamcode.util.auto.AutoSide;

@TeleOp(name = "Auto: Red Far", group = "Autonomous")
public class AutoRedFar extends AutoBase {
    @Override
    protected AutoSide getSide() {
        return AutoSide.FAR;
    }

    @Override
    protected AutoColor getColor() {
        return AutoColor.RED;
    }
}
