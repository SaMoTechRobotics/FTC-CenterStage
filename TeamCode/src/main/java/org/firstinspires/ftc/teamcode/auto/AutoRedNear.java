package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.util.auto.AutoColor;
import org.firstinspires.ftc.teamcode.util.auto.AutoSide;

@Autonomous(name = "Auto: Red Near", group = "Autonomous")
@Disabled
public class AutoRedNear extends AutoBase {
    public AutoRedNear() {
        super(AutoColor.RED, AutoSide.NEAR);
    }
}
