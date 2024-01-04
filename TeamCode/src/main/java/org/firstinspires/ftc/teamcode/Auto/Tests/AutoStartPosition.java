package org.firstinspires.ftc.teamcode.Auto.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Util.Classes.AutoRobot;
import org.firstinspires.ftc.teamcode.Util.Classes.Storage.RobotStorage;
import org.firstinspires.ftc.teamcode.Util.Enums.AutoColor;
import org.firstinspires.ftc.teamcode.Util.Enums.AutoSide;

@Config
@Autonomous(name = "AutoStartPosition", group = "Tests")
public abstract class AutoStartPosition extends LinearOpMode {
    private AutoRobot robot;

    public static AutoSide SIDE = AutoSide.FAR;
    public static AutoColor COLOR = AutoColor.BLUE;

    @Override
    public void runOpMode() {
        Pose2d startPose = RobotStorage.getStartPose(SIDE, COLOR);
        RobotStorage.setPose(startPose);
        robot = new AutoRobot(hardwareMap, telemetry, RobotStorage.pose);

        int c = COLOR.value;
        double outRot = COLOR == AutoColor.BLUE ? 270 : 90;

        waitForStart();

        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose)
                        .strafeToLinearHeading(new Vector2d(-36, 60 * c), Math.toRadians(outRot))
                        .waitSeconds(5)
                        .strafeToLinearHeading(new Vector2d(startPose.position.x, startPose.position.y), Math.toRadians(outRot))
                        .build()
        );

        while (opModeIsActive()) {
            idle();
        }
    }
}