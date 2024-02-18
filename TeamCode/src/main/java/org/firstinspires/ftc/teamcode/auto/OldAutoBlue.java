package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.util.auto.AutoColor;
import org.firstinspires.ftc.teamcode.util.auto.AutoSide;

@Autonomous(name = "AutoBlue")
public class OldAutoBlue extends OldBaseAuto {
    @Override
    public void setConstants() {
        SIDE = AutoSide.FAR;
        COLOR = AutoColor.BLUE;
    }
}
