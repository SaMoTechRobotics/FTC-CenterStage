package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.util.auto.AutoColor;
import org.firstinspires.ftc.teamcode.util.auto.AutoSide;

@Autonomous(name = "Auto: Blue Near", group = "Autonomous")
public class AutoBlueNear extends AutoBase {
    @Override
    protected AutoSide getSide() {
        return AutoSide.NEAR;
    }

    @Override
    protected AutoColor getColor() {
        return AutoColor.BLUE;
    }
}
