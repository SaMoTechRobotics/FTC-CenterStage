package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.util.auto.AutoColor;
import org.firstinspires.ftc.teamcode.util.auto.AutoSide;

@Autonomous(name = "Auto: Red Near", group = "Autonomous")
public class AutoRedNear extends AutoBase {
    public AutoRedNear() {
        super(AutoColor.RED, AutoSide.NEAR);
    }
}
