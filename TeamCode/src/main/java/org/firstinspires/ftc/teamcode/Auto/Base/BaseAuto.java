package org.firstinspires.ftc.teamcode.Auto.Base;

import com.acmerobotics.dashboard.config.Config;
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

import java.util.Timer;

@Config
public abstract class BaseAuto extends LinearOpMode {
    private AutoRobot robot;

    private final static Boolean Debug = true;

    protected static AutoSide SIDE = AutoSide.RIGHT;
    protected static AutoColor COLOR = AutoColor.RED;

    protected abstract void setConstants();

    public static Double CrossFieldY = 13.0;

    public static Double[] PrepDeliverY = new Double[]{38.0, 33.0, 32.0};
    public static Double[] DeliverY = new Double[]{40.0, 33.0, 28.0};
    public static Double PrepDeliverX = 34.0;
    public static Double DeliverX = 42.0;
    public static Double ExtraDeliverX = 44.0;

    public static Vector2d BoardDeliveryPos = new Vector2d(38, 24);
    public static Vector2d ParkPositionPos = new Vector2d(54, 13);

    BoardPosition boardPosition = BoardPosition.CENTER;

    @Override
    public void runOpMode() {
        setConstants();

        RobotStorage.reset(SIDE, COLOR);
        robot = new AutoRobot(hardwareMap, telemetry, RobotStorage.pose);

        robot.vision.startProcessor(VisionProcessor.SPIKE_LOCATION_DETECTION);
        robot.vision.setColor(COLOR);

        if (Debug) {
            robot.vision.visionPortal.resumeStreaming();
        }

        ElapsedTime timer = new ElapsedTime();

        Pose2d v = new Pose2d(1, COLOR.value, COLOR.value);
        Timer t = new Timer();

        while (!isStarted()) {
            telemetry.addLine("Color: " + COLOR + " Side: " + SIDE);
            telemetry.addData("Status", "Initialized for " + Math.round(timer.seconds()));
            boardPosition = robot.vision.getSpikeLocation();
            telemetry.addData("Spike Location", boardPosition.toString());
            telemetry.update();
        }

        waitForStart();

        robot.vision.stopProcessors();

        robot.claw.close();

        t.schedule(
                new java.util.TimerTask() {
                    @Override
                    public void run() {
                        robot.arm.setRotation(ArmRotation.AutoHoldDown);
                    }
                },
                200
        );

        if (SIDE == AutoSide.RIGHT) {
            switch (boardPosition) {
                case LEFT:
                    Actions.runBlocking(
                            robot.drive.actionBuilder(robot.drive.pose)
                                    .strafeToLinearHeading(new Vector2d(-38, 48), Math.toRadians(270))
                                    .strafeToLinearHeading(new Vector2d(-36, 42), Math.toRadians(270))
                                    .turn(Math.toRadians(60))
                                    .lineToX(-35)
                                    .build()
                    );
                    robot.claw.openNext();
                    Actions.runBlocking(
                            robot.drive.actionBuilder(robot.drive.pose)
                                    .lineToX(-36)
                                    .turn(Math.toRadians(-60))
                                    .strafeToLinearHeading(new Vector2d(-36, 42), Math.toRadians(270))
                                    .strafeToLinearHeading(new Vector2d(-36, CrossFieldY), Math.toRadians(270))
                                    .build()
                    );
                    break;
                case CENTER:
                    Actions.runBlocking(
                            robot.drive.actionBuilder(robot.drive.pose)
                                    .strafeToLinearHeading(new Vector2d(-40, 54), Math.toRadians(270))
                                    .turnTo(Math.toRadians(90))
                                    .strafeToLinearHeading(new Vector2d(-40, 17), Math.toRadians(90))
                                    .build()
                    );
                    robot.claw.openNext();
                    Actions.runBlocking(
                            robot.drive.actionBuilder(robot.drive.pose)
                                    .lineToY(14)
                                    .build()
                    );
                    break;
                case RIGHT:
                    Actions.runBlocking(
                            robot.drive.actionBuilder(robot.drive.pose)
                                    .strafeToLinearHeading(new Vector2d(-42, 48), Math.toRadians(270))
                                    .turn(Math.toRadians(-30))
                                    .lineToY(41)
                                    .build()
                    );
                    robot.claw.openNext();
                    Actions.runBlocking(
                            robot.drive.actionBuilder(robot.drive.pose)
                                    .lineToY(48)
                                    .turn(Math.toRadians(30))
                                    .strafeToLinearHeading(new Vector2d(-36, 48), Math.toRadians(270))
                                    .strafeToLinearHeading(new Vector2d(-36, CrossFieldY), Math.toRadians(270))
                                    .build()
                    );
                    break;
            }
        } else {
            switch (boardPosition) {
                case LEFT:
                    Actions.runBlocking(
                            robot.drive.actionBuilder(robot.drive.pose)
                                    .strafeToLinearHeading(new Vector2d(-36, 36), Math.toRadians(90))
                                    .strafeToLinearHeading(new Vector2d(-32, 34), Math.toRadians(30))
                                    .build()
                    );
                    robot.claw.openNext();
                    break;
                case CENTER:
                    Actions.runBlocking(
                            robot.drive.actionBuilder(robot.drive.pose)
                                    .strafeToLinearHeading(new Vector2d(-36, 24), Math.toRadians(90))
                                    .build()
                    );
                    robot.claw.openNext();
                    break;
                case RIGHT:
                    Actions.runBlocking(
                            robot.drive.actionBuilder(robot.drive.pose)
                                    .lineToY(28)
                                    .turn(Math.toRadians(30))
                                    .build()
                    );
                    robot.claw.openNext();
                    break;
            }
        }

        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose)
//                        .strafeToLinearHeading(new Vector2d(-36, 36), Math.toRadians(90))
//                        .strafeToLinearHeading(new Vector2d(-36, CrossFieldY), Math.toRadians(180))
//                        .lineToY(CrossFieldY)
//                        .turnTo(Math.toRadians(180))
//                        .turn(Math.toRadians(-90))
                        .strafeToLinearHeading(new Vector2d(-36, CrossFieldY), Math.toRadians(180))
                        .build()
        );

        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose)
                        .strafeToLinearHeading(new Vector2d(-35.5, CrossFieldY), Math.toRadians(180))
                        .build()
        );

//        Actions.runBlocking(
//                robot.drive.actionBuilder(robot.drive.pose)
//                        .strafeToLinearHeading(new Vector2d(-40, 20), Math.toRadians(90))
//                        .build()
//        );

        Double boardDeliverY = DeliverY[1];
        Double prepBoardDeliverY = PrepDeliverY[1];

        switch (boardPosition) {
            case LEFT:
                boardDeliverY = DeliverY[0];
                prepBoardDeliverY = PrepDeliverY[0];
                break;
            case RIGHT:
                boardDeliverY = DeliverY[2];
                prepBoardDeliverY = PrepDeliverY[2];
                break;
        }

        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose)
                        .setReversed(true)
                        .strafeToLinearHeading(new Vector2d(0, CrossFieldY), Math.toRadians(180))
                        .strafeToLinearHeading(new Vector2d(PrepDeliverX, CrossFieldY), Math.toRadians(180))
                        .strafeToLinearHeading(new Vector2d(PrepDeliverX, boardDeliverY), Math.toRadians(180))
                        .build()
        );

//
//        Actions.runBlocking(
//                robot.drive.actionBuilder(robot.drive.pose)
//                        .strafeToLinearHeading(new Vector2d(PrepDeliverX, 36), Math.toRadians(180))
//                        .build()
//        );

        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose)
                        .strafeToLinearHeading(new Vector2d(DeliverX, prepBoardDeliverY), Math.toRadians(180))
                        .build()
        );

        robot.arm.setRotation(ArmRotation.LowDeliver);
        robot.arm.setGlobalWristRotation(true);
        robot.arm.update();

        Actions.runBlocking(
                new SleepAction(2)
        );

        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose)
                        .strafeToLinearHeading(new Vector2d(DeliverX, boardDeliverY), Math.toRadians(180))
                        .build()
        );

        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose)
                        .strafeToLinearHeading(new Vector2d(ExtraDeliverX, boardDeliverY), Math.toRadians(180))
                        .build()
        );


//                new ParallelAction(
//                        telemetryPacket -> {
//                            robot.arm.update();
//                            return false;
//                        }
//                false)

        robot.arm.update();

        Actions.runBlocking(
                new SleepAction(0.5)
        );

        robot.claw.open();

        Actions.runBlocking(
                new SleepAction(1)
        );

        robot.arm.setRotation(ArmRotation.Down);
        robot.arm.setWristRotation(WristRotation.Down);

        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose)
                        .setReversed(true)
                        .strafeToLinearHeading(new Vector2d(robot.drive.pose.position.x, ParkPositionPos.y), Math.toRadians(180))
                        .strafeToLinearHeading(new Vector2d(ParkPositionPos.x, ParkPositionPos.y), Math.toRadians(180))
                        .build()
        );
    }
}

