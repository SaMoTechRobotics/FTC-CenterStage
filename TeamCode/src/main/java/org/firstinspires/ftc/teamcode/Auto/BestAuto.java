package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Util.Classes.Robot;

@Autonomous(name = "Best Auto")
public class BestAuto extends LinearOpMode {
    public static Double speed = 0.2;
    public static long time = 5000;

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, telemetry);

        while (!isStarted()) {
            telemetry.addLine("Ready to see the best auto ever?!!?!");
            telemetry.update();
        }

        waitForStart();

        robot.chassis.setPowerAllMotors(speed);

        sleep(time);

        robot.chassis.setPowerAllMotors(0.0);


    }
}

