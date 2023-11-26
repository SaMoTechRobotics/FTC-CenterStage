package org.firstinspires.ftc.teamcode.Auto.Tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Util.Classes.AutoRobot;
import org.firstinspires.ftc.teamcode.Util.Enums.BoardPosition;
import org.firstinspires.ftc.teamcode.Util.Enums.VisionProcessor;

@Autonomous(name = "PlacePixelTest", group = "Tests")
public class PlacePixelTest extends LinearOpMode {
    AutoRobot robot;

    @Override
    public void runOpMode() {
        robot = new AutoRobot(hardwareMap, telemetry);
        robot.vision.startProcessor(VisionProcessor.APRIL_TAG_DETECTION);

        waitForStart();

        while (opModeIsActive()) {
            if (robot.vision.isBoardDetected(BoardPosition.CENTER)) {
                telemetry.addLine("Detected");
                Pose2d pose = robot.vision.getBoardAlignedPose(robot.drive.pose, BoardPosition.CENTER);
                Pose2d relPose = new Pose2d(
                        robot.drive.pose.position.x + pose.position.x,
                        robot.drive.pose.position.y + pose.position.y,
                        robot.drive.pose.heading.real + pose.heading.real
                );
                telemetry.addData("X", pose.position.x);
                telemetry.addData("Y", pose.position.y);
                telemetry.addData("Heading", Math.toDegrees(pose.heading.log()));
                telemetry.update();
                if (gamepad1.a) {
                    Actions.runBlocking(
                            robot.drive.actionBuilder(robot.drive.pose)
                                    .strafeToLinearHeading(new Vector2d(pose.position.x, pose.position.y), pose.heading)
                                    .build());
                }
            }
        }
    }
}
