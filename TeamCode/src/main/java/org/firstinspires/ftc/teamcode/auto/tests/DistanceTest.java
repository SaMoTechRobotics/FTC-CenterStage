package org.firstinspires.ftc.teamcode.auto.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.auto.RobotStorage;
import org.firstinspires.ftc.teamcode.util.robot.AutoRobot;

@Config
@Autonomous(name = "DistanceTest", group = "Tests")
public class DistanceTest extends LinearOpMode {

    private AutoRobot robot;

    public static double DISTANCE = 48.0;

    @Override
    public void runOpMode() {
        RobotStorage.reset();
        robot = new AutoRobot(hardwareMap, RobotStorage.pose);

        ElapsedTime timer = new ElapsedTime();

        waitForStart();

        Actions.runBlocking(
                robot.drive
                        .actionBuilder(robot.drive.pose)
                        .strafeToLinearHeading(new Vector2d(DISTANCE, 0), 0)
                        .build()
        );

        Actions.runBlocking(
                robot.drive
                        .actionBuilder(robot.drive.pose)
                        .strafeToLinearHeading(new Vector2d(DISTANCE, 0), 0)
                        .build()
        );

        while (opModeIsActive() && !isStopRequested()) {
            idle();
        }
    }
}
