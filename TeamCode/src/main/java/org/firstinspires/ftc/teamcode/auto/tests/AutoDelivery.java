package org.firstinspires.ftc.teamcode.auto.tests;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.util.auto.BoardPosition;
import org.firstinspires.ftc.teamcode.util.lib.GamepadButton;
import org.firstinspires.ftc.teamcode.util.lib.StatefulGamepad;
import org.firstinspires.ftc.teamcode.util.robot.AutoRobot;
import org.firstinspires.ftc.teamcode.util.robot.arm.ArmRotation;
import org.firstinspires.ftc.teamcode.util.robot.chassis.ChassisSpeed;
import org.firstinspires.ftc.teamcode.util.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.Optional;

@Config
@Autonomous(name = "AutoDelivery", group = "Tests")
public class AutoDelivery extends LinearOpMode {
    AutoRobot robot;

    public static BoardPosition Side = BoardPosition.CENTER;

    public static double xMargin = 1.0;
    public static double yMargin = 1.0;
    public static double rotMargin = 1.0;

    public static double xCoefficient = 0.01;
    public static double yCoefficient = 0.01;
    public static double rotCoefficient = 0.001;

    public static double yDist = 20.0;

    @Override
    public void runOpMode() {
        robot = new AutoRobot(hardwareMap, new Pose2d(0, 0, 0));
        robot.vision.startProcessor(VisionProcessor.APRIL_TAG_DETECTION);

        boolean autoPlace = false;

        boolean aligned = false;

        waitForStart();

        StatefulGamepad gamepad1Buttons = new StatefulGamepad(gamepad1);

        while (opModeIsActive() && !isStopRequested()) {
            gamepad1Buttons.update();

            if (gamepad1Buttons.wasJustReleased(GamepadButton.Y)) {
                autoPlace = !autoPlace;
                aligned = false;
                if (autoPlace) {
                    robot.arm.setRotation(ArmRotation.AutoDeliver);
                } else {
                    robot.arm.setRotation(ArmRotation.Down);
                }
            }

            if (!autoPlace) {
                robot.drive.setDrivePowers(new PoseVelocity2d(
                        new Vector2d(-ChassisSpeed.applyDeadZone(gamepad1.left_stick_y, ChassisSpeed.DriveDeadZone),
                                -ChassisSpeed.applyDeadZone(gamepad1.left_stick_x, ChassisSpeed.StrafeDeadZone)),
                        -ChassisSpeed.applyDeadZone(gamepad1.right_stick_x, ChassisSpeed.TurnDeadZone)
                ));

                if (gamepad1Buttons.wasJustPressed(GamepadButton.RIGHT_BUMPER)) {
                    robot.claw.close();
                } else if (gamepad1Buttons.wasJustPressed(GamepadButton.LEFT_BUMPER)) {
                    robot.claw.open();
                }
            } else {
                robot.arm.update();
                if (!aligned) {
                    aligned = alignWithBoard(Side);
                } else {
                    Actions.runBlocking(new SleepAction(0.5));
                    robot.claw.open();
                    Actions.runBlocking(new SleepAction(0.5));
                    robot.arm.setRotation(ArmRotation.Down);
                    autoPlace = false;
                    aligned = false;
                }
            }
        }
    }

    private AprilTagPoseFtc lastPose = null;

    private boolean alignWithBoard(BoardPosition side) {
        if (robot.vision.isBoardDetected(Side) || lastPose != null) {
            Optional<AprilTagPoseFtc> tag = robot.vision.getBoardPose(Side);
            AprilTagPoseFtc pose = lastPose;
            if (tag.isPresent()) pose = tag.get();
            lastPose = pose;

            double targetY = pose.y - yDist;

            double xPower = Math.abs(pose.x) > xMargin ? xCoefficient * pose.x : 0;
            double yPower = Math.abs(targetY) > yMargin ? yCoefficient * targetY : 0;
            double rotPower = Math.abs(pose.yaw) > rotMargin ? rotCoefficient * pose.yaw : 0;

            robot.drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(-xPower, -yPower),
                    -rotPower
            ));
            return xPower == 0 && yPower == 0 && rotPower == 0;
        } else {
            robot.drive.setDrivePowers(new PoseVelocity2d(
                    new Vector2d(0, 0.5),
                    0
            ));
        }
        return false;
    }
}
