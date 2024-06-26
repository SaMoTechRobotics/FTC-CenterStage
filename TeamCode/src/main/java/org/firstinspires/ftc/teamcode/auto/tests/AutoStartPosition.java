package org.firstinspires.ftc.teamcode.auto.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.util.auto.AutoColor;
import org.firstinspires.ftc.teamcode.util.auto.AutoSide;
import org.firstinspires.ftc.teamcode.util.auto.RobotStorage;
import org.firstinspires.ftc.teamcode.util.robot.AutoRobot;

@Config
@Autonomous(name = "AutoStartPosition", group = "Tests")
public class AutoStartPosition extends LinearOpMode {

    public static AutoSide SIDE = AutoSide.FAR;
    public static AutoColor COLOR = AutoColor.RED;

    @Override
    public void runOpMode() {
        Pose2d startPose = RobotStorage.getStartPose(SIDE, COLOR);
        RobotStorage.setPose(startPose);
        AutoRobot robot = new AutoRobot(hardwareMap, RobotStorage.pose);

        int c = COLOR.value;
        double outRot = COLOR == AutoColor.BLUE ? 270 : 90;

        while (!isStarted()) {
            idle();
        }

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