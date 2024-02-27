package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.util.auto.AutoColor;
import org.firstinspires.ftc.teamcode.util.auto.AutoSide;

@Autonomous(name = "Auto: Blue Far", group = "Autonomous")
public class AutoBlueFar extends AutoBase {
    public AutoBlueFar() {
        super(AutoColor.BLUE, AutoSide.FAR);
    }
}