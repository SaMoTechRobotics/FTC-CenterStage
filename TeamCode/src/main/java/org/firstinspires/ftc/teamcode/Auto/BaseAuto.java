package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Util.Classes.Robot.Robot;
import org.firstinspires.ftc.teamcode.Util.Enums.AutoSide;
import org.firstinspires.ftc.teamcode.Util.Classes.Robot.RobotStorage;

@Config
public class BaseAuto extends LinearOpMode {
    public final static AutoSide SIDE = AutoSide.Right;

    @Override
    public void runOpMode() throws InterruptedException {
        RobotStorage.reset();
        Robot robot = new Robot(hardwareMap);

        while (!isStarted()) {
        }

        waitForStart();
        if (isStopRequested()) return;


    }
}

