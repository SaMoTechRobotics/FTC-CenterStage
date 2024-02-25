package org.firstinspires.ftc.teamcode.auto.old;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.util.auto.AutoColor;
import org.firstinspires.ftc.teamcode.util.auto.AutoSide;

@Autonomous(name = "AutoBlue")
@Disabled
public class OldAutoBlue extends OldBaseAuto {
    @Override
    public void setConstants() {
        SIDE = AutoSide.FAR;
        COLOR = AutoColor.BLUE;
    }
}
