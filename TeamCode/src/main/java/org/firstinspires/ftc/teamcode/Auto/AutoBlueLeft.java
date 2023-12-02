package org.firstinspires.ftc.teamcode.Auto;

import org.firstinspires.ftc.teamcode.Auto.Base.BaseAuto;
import org.firstinspires.ftc.teamcode.Util.Enums.AutoColor;
import org.firstinspires.ftc.teamcode.Util.Enums.AutoSide;

//@Autonomous(name = "AutoBlueLeft", group = "Auto")
public class AutoBlueLeft extends BaseAuto {
    @Override
    protected void setConstants() {
        SIDE = AutoSide.LEFT;
        COLOR = AutoColor.BLUE;
    }
}

