package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Auto.Base.BaseAuto;
import org.firstinspires.ftc.teamcode.Util.Enums.AutoColor;
import org.firstinspires.ftc.teamcode.Util.Enums.AutoSide;

@Autonomous(name = "AutoRed", group = "Auto")
public class AutoRedLeft extends BaseAuto {
    @Override
    protected void setConstants() {
        SIDE = AutoSide.LEFT;
        COLOR = AutoColor.RED;
    }
}

