package org.firstinspires.ftc.teamcode.auto.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.auto.AutoColor;
import org.firstinspires.ftc.teamcode.util.auto.AutoSide;
import org.firstinspires.ftc.teamcode.util.auto.RobotStorage;
import org.firstinspires.ftc.teamcode.util.robot.AutoRobot;

@Config
@Autonomous(name = "TrajectoryTesting", group = "Tests")
public class TrajectoryTesting extends LinearOpMode {
    private AutoRobot robot;

    protected static AutoSide SIDE = AutoSide.FAR;
    protected static AutoColor COLOR = AutoColor.BLUE;

    @Override
    public void runOpMode() {
        RobotStorage.setPose(RobotStorage.getStartPose(SIDE, COLOR));
        robot = new AutoRobot(hardwareMap, RobotStorage.pose);

        ElapsedTime timer = new ElapsedTime();

        while (!isStarted()) {
            telemetry.addLine("Testing Trajectory for Color: " + COLOR + " Side: " + SIDE);
            telemetry.addData("Status", "Initialized for " + Math.round(timer.seconds()));
            telemetry.update();
        }

        Pose2d v = new Pose2d(1, COLOR.value, COLOR.value);

        waitForStart();

        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose)
                        .lineToY(12 /*velocity constraint here*/)
                        .turn(Math.toRadians(90))
                        .build()
        );

        Actions.runBlocking(
                robot.drive.actionBuilder(new Pose2d(robot.drive.pose.position.x, 12, Math.toRadians(180)))
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(0, 12, Math.toRadians(180)), Math.toRadians(0))
                        .splineToLinearHeading(new Pose2d(30, 12, Math.toRadians(180)), Math.toRadians(0))
                        .strafeToLinearHeading(new Vector2d(40, 36), Math.toRadians(180))
                        .build()
        );

//        Actions.runBlocking(
//                robot.drive.actionBuilder(robot.drive.pose)
//                        .splineToLinearHeading(new Pose2d(-40, 12, Math.toRadians(0)), Math.toRadians(0))
//                        .lineToX(-30)
//                        .build()
//        );
//
//
//        Actions.runBlocking(
//                robot.drive.actionBuilder(robot.drive.pose)
//                        .strafeToLinearHeading(new Vector2d(-40, 20), Math.toRadians(90))
//                        .build()
//        );
    }
}

