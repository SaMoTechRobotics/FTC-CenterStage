package org.firstinspires.ftc.teamcode.Auto;

import org.firstinspires.ftc.teamcode.Auto.Base.BaseAuto;
import org.firstinspires.ftc.teamcode.Util.Enums.AutoColor;
import org.firstinspires.ftc.teamcode.Util.Enums.AutoSide;

//@Autonomous(name = "AutoRedRight", group = "Auto")
public class AutoRedRight extends BaseAuto {
    @Override
    protected void setConstants() {
        SIDE = AutoSide.RIGHT;
        COLOR = AutoColor.RED;
    }
}