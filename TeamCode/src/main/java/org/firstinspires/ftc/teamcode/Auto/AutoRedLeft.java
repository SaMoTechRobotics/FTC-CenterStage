package org.firstinspires.ftc.teamcode.Auto;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Auto.Base.BaseAuto;
import org.firstinspires.ftc.teamcode.Util.Enums.AutoColor;
import org.firstinspires.ftc.teamcode.Util.Enums.AutoSide;

@Autonomous(name = "AutoRedLeft", group = "Auto")
public class AutoRedLeft extends BaseAuto {
    AutoSide SIDE = AutoSide.LEFT;
    AutoColor COLOR = AutoColor.RED;
}

