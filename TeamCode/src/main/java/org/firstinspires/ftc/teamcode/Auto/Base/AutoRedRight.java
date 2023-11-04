package org.firstinspires.ftc.teamcode.Auto.Base;

import org.firstinspires.ftc.teamcode.Util.Enums.AutoColor;
import org.firstinspires.ftc.teamcode.Util.Enums.AutoSide;

public class AutoRedRight extends BaseAuto {
    AutoSide SIDE = AutoSide.RIGHT;
    AutoColor COLOR = AutoColor.RED;

    @Override
    public void runOpMode() {
        super.initAuto();

        while(!isStarted()) {

            telemetry.addData("Status", "Waiting for start");
            telemetry.update();
        }
    }
}
