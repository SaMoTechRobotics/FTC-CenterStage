package org.firstinspires.ftc.teamcode.Auto.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Util.Classes.AutoRobot;
import org.firstinspires.ftc.teamcode.Util.Classes.Lib.GamepadButton;
import org.firstinspires.ftc.teamcode.Util.Classes.Lib.StatefulGamepad;
import org.firstinspires.ftc.teamcode.Util.Constants.Auto.BoardAlignmentConstants;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.ArmRotation;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.ChassisSpeed;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.WristRotation;
import org.firstinspires.ftc.teamcode.Util.Enums.BoardPosition;
import org.firstinspires.ftc.teamcode.Util.Enums.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.lang.Math;
import java.util.Optional;

@Config
@Autonomous(name = "PlacePixelTest", group = "Tests")
public class PlacePixelTest extends LinearOpMode {
    AutoRobot robot;

    public static boolean AutoVersion = true;
    public static boolean CustomDrive = false;

    public static BoardPosition Side = BoardPosition.CENTER;

    public static Double xError = 1.0;
    public static Double yError = 1.0;
    public static Double rotError = 1.0;

    public static Double yOff = 20.0;

    public static Double xSpeed = 0.2;
    public static Double slowXSpeed = 0.1;
    public static Double ySpeed = 0.2;
    public static Double slowYSpeed = 0.1;

    public static Double rotSpeed = 0.2;
    public static Double slowRotMargin = 10.0;
    public static Double slowRotSpeed = 0.08;

    @Override
    public void runOpMode() {
        robot = new AutoRobot(hardwareMap, telemetry, new Pose2d(0, 0, 0));
        robot.vision.startProcessor(VisionProcessor.APRIL_TAG_DETECTION);

        waitForStart();

        StatefulGamepad gamepad1Buttons = new StatefulGamepad(gamepad1);

        if (!AutoVersion) {
            while (opModeIsActive()) {
                if (gamepad1Buttons.wasJustReleased(GamepadButton.B)) {
                    robot.arm.setRotation(ArmRotation.Down);
                    robot.arm.setWristRotation(WristRotation.Down);
                } else if (gamepad1Buttons.wasJustReleased(GamepadButton.Y)) {
                    robot.arm.setRotation(ArmRotation.LowDeliver);
                    robot.arm.setGlobalWristRotation(true);
                    robot.arm.update();
                }

                gamepad1Buttons.update();

                if (!CustomDrive) {
                    double drivePower = Math.abs(gamepad1.left_stick_y) > ChassisSpeed.JoystickYMargin ? gamepad1.left_stick_y : 0;
                    double strafePower = Math.abs(gamepad1.left_stick_x) > ChassisSpeed.JoystickXMargin ? gamepad1.left_stick_x : 0;
                    double turnPower = Math.abs(gamepad1.right_stick_x) > ChassisSpeed.JoystickXMargin ? gamepad1.right_stick_x : 0;

                    boolean anyActiveJoystick = drivePower != 0 || strafePower != 0 || turnPower != 0;

                    robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-drivePower, -strafePower), -turnPower));
                    if (robot.vision.isBoardDetected(Side)) {
                        telemetry.addLine("Detected");
                        Pose2d pose = robot.vision.getBoardAlignedPose(robot.drive.pose, Side);
                        Pose2d relPose = new Pose2d(
                                robot.drive.pose.position.x + pose.position.x,
                                robot.drive.pose.position.y + pose.position.y,
                                robot.drive.pose.heading.real + pose.heading.real
                        );
//                        telemetry.addData("Rel X", pose.position.x);
//                        telemetry.addData("Rel Y", pose.position.y);
//                        telemetry.addData("Rel Heading", Math.toDegrees(pose.heading.log()));
//                        telemetry.addLine("---");
                        Optional<AprilTagPoseFtc> tpose = robot.vision.getBoardPose(Side);
                        if (tpose.isPresent()) {
                            telemetry.addData("X", tpose.get().x);
                            telemetry.addData("Y", tpose.get().y - 20);
                            telemetry.addData("Heading", tpose.get().yaw);
                            telemetry.update();
                        }
                        if (gamepad1.a && tpose.isPresent()) {
                            Vector2d newPose = new Vector2d(robot.drive.pose.position.x + (tpose.get().y - 20), robot.drive.pose.position.y - tpose.get().x);
                            Rotation2d heading = robot.drive.pose.heading.plus(Math.toRadians(tpose.get().yaw));

                            Actions.runBlocking(
                                    robot.drive.actionBuilder(robot.drive.pose)
                                            .strafeToLinearHeading(newPose, heading)
                                            .build());

//                            Actions.runBlocking(
//                                    robot.drive.actionBuilder(robot.drive.pose)
//                                            .strafeToLinearHeading(newPose, heading)
//                                            .build());

//                        Actions.runBlocking(
//                                robot.drive.actionBuilder(robot.drive.pose)
//                                        .turn(Math.toRadians(tpose.get().yaw))
//                                        .strafeTo(new Vector2d(robot.drive.pose.position.x + tpose.get().x, robot.drive.pose.position.y))
////                                        .turnTo(pose.heading)
////                                        .strafeToLinearHeading(new Vector2d(pose.position.x, pose.position.y), pose.heading)
//                                        .build());
                        }
                    }
                } else {
                    if (robot.vision.isBoardDetected(Side)) {
                        Optional<AprilTagPoseFtc> boardPose = robot.vision.getBoardPose(Side);
                        if (boardPose.isPresent()) {
                            AprilTagPoseFtc pose = boardPose.get();
                            telemetry.addLine("Detected");
                            telemetry.addData("X", pose.x);
                            telemetry.addData("Y", pose.y);
                            telemetry.addData("Heading", pose.yaw);
                            if (gamepad1.a) {
                                if (Math.abs(pose.yaw) > rotError) {
                                    double rvel = rotSpeed;
                                    if (Math.abs(pose.yaw) < slowRotMargin) rvel = slowRotSpeed;
                                    if (pose.yaw < 0) rvel *= -1;
                                    robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), rvel));
                                    telemetry.addLine("Rotating");
                                } else if (Math.abs(pose.x) > xError) {
                                    double xvel = xSpeed;
                                    if (Math.abs(pose.x) < 3) xvel = slowXSpeed;
                                    if (pose.x > 0) xvel *= -1;
                                    robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, -xvel), 0));
                                    telemetry.addLine("Lining up x");
                                } else if (Math.abs(pose.y - yOff) > yError) {
                                    double yvel = ySpeed;
                                    if (Math.abs(pose.y) < 2) yvel = slowYSpeed;
                                    if (pose.y < yOff) yvel *= -1;
                                    robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-yvel, 0), 0));
                                    telemetry.addLine("Lining up y");
                                } else {
                                    robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
                                    telemetry.addLine("Done");
                                }
                            }
                            telemetry.update();
                        }
                    }
                    if (!gamepad1.a) {
                        double drivePower = Math.abs(gamepad1.left_stick_y) > ChassisSpeed.JoystickYMargin ? gamepad1.left_stick_y : 0;
                        double strafePower = Math.abs(gamepad1.left_stick_x) > ChassisSpeed.JoystickXMargin ? gamepad1.left_stick_x : 0;
                        double turnPower = Math.abs(gamepad1.right_stick_x) > ChassisSpeed.JoystickXMargin ? gamepad1.right_stick_x : 0;

                        boolean anyActiveJoystick = drivePower != 0 || strafePower != 0 || turnPower != 0;

                        robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-drivePower, -strafePower), -turnPower));
                    }
                }
            }
        } else {
            Actions.runBlocking(
                    robot.drive.actionBuilder(robot.drive.pose)
                            .turn(Math.toRadians(180))
                            .build());

            robot.arm.setRotation(ArmRotation.LowDeliver);
            robot.arm.setGlobalWristRotation(true);
            robot.arm.update();

            ElapsedTime time = new ElapsedTime();

            boolean aligned = false;
            while (opModeIsActive()) {
                if (time.seconds() > 4) break;
                if (robot.vision.isBoardDetected(Side)) {
                    Optional<AprilTagPoseFtc> tpose = robot.vision.getBoardPose(Side);
                    if (tpose.isPresent()) {
                        Vector2d newPose = new Vector2d(robot.drive.pose.position.x + (tpose.get().y - BoardAlignmentConstants.DistFromBoard), robot.drive.pose.position.y - tpose.get().x);
                        Rotation2d heading = robot.drive.pose.heading.plus(Math.toRadians(tpose.get().yaw));

                        Actions.runBlocking(
                                robot.drive.actionBuilder(robot.drive.pose)
                                        .strafeToLinearHeading(newPose, heading, robot.drive.slowVelConstraint)
                                        .build());

//                        Actions.runBlocking(
//                                robot.drive.actionBuilder(robot.drive.pose)
//                                        .strafeToLinearHeading(newPose, heading)
//                                        .waitSeconds(1)
//                                        .build());

                        aligned = true;
                    }
                }
            }

            robot.claw.open();

            Actions.runBlocking(
                    new SleepAction(1)
            );

            robot.arm.setRotation(ArmRotation.Down);
            robot.arm.setWristRotation(WristRotation.Down);

            Actions.runBlocking(
                    robot.drive.actionBuilder(robot.drive.pose)
                            .turn(Math.toRadians(180))
                            .build());

            while (opModeIsActive()) {
                idle();
            }
        }
    }
}