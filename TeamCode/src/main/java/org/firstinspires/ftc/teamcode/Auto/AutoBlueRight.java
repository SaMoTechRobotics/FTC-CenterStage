package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.Auto.Base.BaseAuto;
import org.firstinspires.ftc.teamcode.Util.Enums.AutoColor;
import org.firstinspires.ftc.teamcode.Util.Enums.AutoSide;

@Autonomous(name = "AutoBlueRight", group = "Auto")
public class AutoBlueRight extends BaseAuto {
    AutoSide SIDE = AutoSide.RIGHT;
    AutoColor COLOR = AutoColor.BLUE;
}