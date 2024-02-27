package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.util.auto.AutoColor;
import org.firstinspires.ftc.teamcode.util.auto.AutoSide;

@Autonomous(name = "Auto: Blue Near", group = "Autonomous")
@Disabled
public class AutoBlueNear extends AutoBase {
    public AutoBlueNear() {
        super(AutoColor.BLUE, AutoSide.NEAR);
    }
}
