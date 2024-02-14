package org.firstinspires.ftc.teamcode.auto.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.robot.AutoRobot;
import org.firstinspires.ftc.teamcode.util.auto.RobotStorage;

@Config
@Autonomous(name = "TurnTest", group = "Tests")
public class TurnTest extends LinearOpMode {
    public static double rotation = 360;
    public static boolean toTarget = false;

    @Override
    public void runOpMode() {
        RobotStorage.reset();
        AutoRobot robot = new AutoRobot(hardwareMap, telemetry, RobotStorage.pose);

        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        if (toTarget) {
            Actions.runBlocking(
                    robot.drive.actionBuilder(robot.drive.pose)
                            .turnTo(Math.toRadians(rotation))
                            .build()
            );
        } else {
            Actions.runBlocking(
                    robot.drive.actionBuilder(robot.drive.pose)
                            .turn(Math.toRadians(rotation))
                            .build()
            );
        }
    }
}

