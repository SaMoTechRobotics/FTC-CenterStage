package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Auto.Base.BaseAuto;
import org.firstinspires.ftc.teamcode.Util.Enums.AutoColor;
import org.firstinspires.ftc.teamcode.Util.Enums.AutoSide;

@Autonomous(name = "AutoRedFar", group = "Auto")
public class AutoRedFar extends BaseAuto {
    @Override
    protected void setConstants() {
        SIDE = AutoSide.FAR;
        COLOR = AutoColor.RED;
    }
}

