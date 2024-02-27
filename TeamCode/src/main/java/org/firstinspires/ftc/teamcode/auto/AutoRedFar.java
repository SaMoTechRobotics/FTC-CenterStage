package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.util.auto.AutoColor;
import org.firstinspires.ftc.teamcode.util.auto.AutoSide;

@Autonomous(name = "Auto: Red Far", group = "Autonomous")
public class AutoRedFar extends AutoBase {
    public AutoRedFar() {
        super(AutoColor.RED, AutoSide.FAR);
    }
}
