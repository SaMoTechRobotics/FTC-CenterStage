package org.firstinspires.ftc.teamcode.Auto.Base;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Util.Classes.AutoRobot;
import org.firstinspires.ftc.teamcode.Util.Classes.Storage.RobotStorage;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.ArmRotation;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.WristRotation;
import org.firstinspires.ftc.teamcode.Util.Enums.AutoColor;
import org.firstinspires.ftc.teamcode.Util.Enums.AutoSide;
import org.firstinspires.ftc.teamcode.Util.Enums.BoardPosition;
import org.firstinspires.ftc.teamcode.Util.Enums.VisionProcessor;

@Config
public abstract class BaseAuto extends LinearOpMode {
    private AutoRobot robot;

    private final static Boolean Debug = true;

    protected static AutoSide SIDE = AutoSide.RIGHT;
    protected static AutoColor COLOR = AutoColor.RED;

    protected abstract void setConstants();

    protected static class DriveDistances {
        public static Double ToParking = 50.0;
    }

    public static double testing = 0;

    BoardPosition boardPosition = BoardPosition.CENTER;

    @Override
    public void runOpMode() {
        setConstants();

        RobotStorage.reset(SIDE, COLOR);
        robot = new AutoRobot(hardwareMap, telemetry, RobotStorage.pose);

        robot.vision.startProcessor(VisionProcessor.SPIKE_LOCATION_DETECTION);
        if (Debug) {
            robot.vision.visionPortal.resumeStreaming();
        }

        ElapsedTime timer = new ElapsedTime();

        while (!isStarted()) {
            telemetry.addLine("Color: " + COLOR + " Side: " + SIDE);
            telemetry.addData("Status", "Initialized for " + Math.round(timer.seconds()));
            boardPosition = robot.vision.getSpikeLocation();
            telemetry.addData("Spike Location", boardPosition.toString());
            telemetry.update();
        }

        Pose2d v = new Pose2d(1, COLOR.value, COLOR.value);

        waitForStart();

        robot.arm.setRotation(ArmRotation.HoldDown);

        robot.vision.stopProcessors();

        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose)
                        .splineToLinearHeading(new Pose2d(-40, 12, Math.toRadians(0)).times(v), Math.toRadians(testing))
                        .build()
        );

        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose)
                        .strafeToLinearHeading(new Vector2d(-40, 20), Math.toRadians(90))
                        .build()
        );


        robot.claw.openNext();

        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose)
                        .setReversed(true)
                        .splineToLinearHeading(new Pose2d(0, 12, Math.toRadians(90)).times(v), Math.toRadians(testing))
                        .splineToLinearHeading(new Pose2d(36, 36, Math.toRadians(90)).times(v), Math.toRadians(testing))
                        .strafeToLinearHeading(new Vector2d(40, 36), Math.toRadians(90))
                        .build()
        );

        robot.arm.setRotation(ArmRotation.HighDeliver);
        robot.arm.setGlobalWristRotation(true);

        Actions.runBlocking(
                new ParallelAction(
                        robot.drive.actionBuilder(robot.drive.pose)
                                .setTangent(0)
                                .lineToX(42 * v.position.x)
                                .build(),
                        telemetryPacket -> {
                            robot.arm.update();
                            return false;
                        }
                )
        );

        robot.claw.open();

        Actions.runBlocking(
                new SleepAction(0.5)
        );

        robot.arm.setRotation(ArmRotation.Down);
        robot.arm.setWristRotation(WristRotation.Down);

        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose)
                        .lineToY(12 * v.position.y)
                        .lineToX(60 * v.position.x)
                        .build()
        );
    }
}

