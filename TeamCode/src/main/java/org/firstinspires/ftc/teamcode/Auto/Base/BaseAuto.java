package org.firstinspires.ftc.teamcode.Auto.Base;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Util.Classes.AutoRobot;
import org.firstinspires.ftc.teamcode.Util.Classes.Storage.RobotStorage;
import org.firstinspires.ftc.teamcode.Util.Constants.Auto.BoardAlignmentConstants;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.ArmRotation;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.ChassisSpeed;
import org.firstinspires.ftc.teamcode.Util.Constants.Robot.WristRotation;
import org.firstinspires.ftc.teamcode.Util.Enums.AutoColor;
import org.firstinspires.ftc.teamcode.Util.Enums.AutoSide;
import org.firstinspires.ftc.teamcode.Util.Enums.BoardPosition;
import org.firstinspires.ftc.teamcode.Util.Enums.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.lang.Math;
import java.util.Optional;
import java.util.Timer;

@Config
public abstract class BaseAuto extends LinearOpMode {
    private AutoRobot robot;

    private final static Boolean Debug = true;

    protected static AutoSide SIDE = AutoSide.FAR;
    protected static AutoColor COLOR = AutoColor.RED;

    protected abstract void setConstants();

    public static Double CrossFieldY = 13.0;

    public static Double[] PrepDeliverY = new Double[]{38.0, 33.0, 29.0};
    public static Double[] DeliverY = new Double[]{40.0, 33.0, 28.0};
    public static Double LeftBonus = 1.0;
    public static Double RightBonus = 1.0;
    public static Double PrepDeliverX = 34.0;
    public static Double DeliverX = 38.0;

    public static Vector2d ParkPositionPos = new Vector2d(50, 36);

    public static Double AlignWithBoardTime = 2.0;
    public static Double PushBoardTime = 1.0;

    BoardPosition boardPosition = BoardPosition.CENTER;

    @Override
    public void runOpMode() {
        setConstants();

        Pose2d startPose = RobotStorage.getStartPose(SIDE, COLOR);
        RobotStorage.setPose(startPose);
        robot = new AutoRobot(hardwareMap, telemetry, RobotStorage.pose);
//
        robot.vision.startProcessor(VisionProcessor.SPIKE_LOCATION_DETECTION);
        robot.vision.setColor(COLOR);

        if (Debug) {
            robot.vision.visionPortal.resumeStreaming();
        }

        ElapsedTime timer = new ElapsedTime();

        Vector2d v = new Vector2d(1, COLOR.value);
        int c = COLOR.value;

        double outRot = COLOR == AutoColor.BLUE ? 270 : 90;
        Timer t = new Timer();

        while (!isStarted()) {
            telemetry.addLine("Color: " + COLOR + " Side: " + SIDE);
            telemetry.addData("Status", "Initialized for " + Math.round(timer.seconds()));
            boardPosition = robot.vision.getSpikeLocation();
            telemetry.addData("Spike Location", boardPosition.toString());
            telemetry.update();
        }

        waitForStart();

        robot.vision.close();
//        robot.vision.nextPortal();

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


        if (COLOR == AutoColor.BLUE) {
            switch (boardPosition) {
                case LEFT:
                    Actions.runBlocking(
                            robot.drive.actionBuilder(robot.drive.pose)
                                    .strafeToLinearHeading(new Vector2d(-38, 48 * c), Math.toRadians(outRot))
                                    .strafeToLinearHeading(new Vector2d(-36, 42 * c), Math.toRadians(outRot))
                                    .turn(Math.toRadians(60))
                                    .lineToX(-35)
                                    .build()
                    );
                    robot.claw.openNext();
                    Actions.runBlocking(
                            robot.drive.actionBuilder(robot.drive.pose)
                                    .lineToX(-36)
                                    .turn(Math.toRadians(-60))
                                    .strafeToLinearHeading(new Vector2d(-36, 42 * c), Math.toRadians(outRot))
                                    .strafeToLinearHeading(new Vector2d(-36, CrossFieldY * c), Math.toRadians(outRot))
                                    .build()
                    );
                    break;
                case CENTER:
                    Actions.runBlocking(
                            robot.drive.actionBuilder(robot.drive.pose)
                                    .strafeToLinearHeading(new Vector2d(-38, 54 * c), Math.toRadians(outRot))
                                    .turnTo(Math.toRadians(90))
                                    .build()
                    );
                    Actions.runBlocking(
                            robot.drive.actionBuilder(robot.drive.pose)
                                    .turnTo(Math.toRadians(90))
                                    .strafeToLinearHeading(new Vector2d(-36, 16 * c), Math.toRadians(90))
                                    .build()
                    );
                    robot.claw.openNext();
                    Actions.runBlocking(
                            robot.drive.actionBuilder(robot.drive.pose)
                                    .lineToY(12 * c)
                                    .build()
                    );
                    break;
                case RIGHT:
                    Actions.runBlocking(
                            robot.drive.actionBuilder(robot.drive.pose)
                                    .strafeToLinearHeading(new Vector2d(-44
                                            , 47 * c), Math.toRadians(outRot))
                                    .turn(Math.toRadians(-30))
                                    .lineToY(40.5 * c)
                                    .build()
                    );
                    robot.claw.openNext();
                    Actions.runBlocking(
                            robot.drive.actionBuilder(robot.drive.pose)
                                    .lineToY(47 * c)
                                    .turn(Math.toRadians(30))
                                    .strafeToLinearHeading(new Vector2d(-36, 47 * c), Math.toRadians(outRot))
                                    .strafeToLinearHeading(new Vector2d(-36, CrossFieldY * c), Math.toRadians(outRot))
                                    .build()
                    );
                    break;
            }
        } else { // red placement
            switch (boardPosition) {
                case LEFT:
                    Actions.runBlocking(
                            robot.drive.actionBuilder(robot.drive.pose)
                                    .strafeToLinearHeading(new Vector2d(-41, 45 * c), Math.toRadians(outRot))
                                    .turn(Math.toRadians(30))
                                    .lineToY(41 * c)
                                    .build()
                    );
                    robot.claw.openNext();
                    Actions.runBlocking(
                            robot.drive.actionBuilder(robot.drive.pose)
                                    .lineToY(45 * c)
                                    .turn(Math.toRadians(-30))
                                    .strafeToLinearHeading(new Vector2d(-36, 45 * c), Math.toRadians(outRot))
                                    .strafeToLinearHeading(new Vector2d(-36, CrossFieldY * c), Math.toRadians(outRot))
                                    .build()
                    );
                    break;
                case CENTER:
                    Actions.runBlocking(
                            robot.drive.actionBuilder(robot.drive.pose)
                                    .strafeToLinearHeading(new Vector2d(-36, 54 * c), Math.toRadians(outRot))
                                    .turnTo(Math.toRadians(270))
                                    .build()
                    );
                    Actions.runBlocking(
                            robot.drive.actionBuilder(robot.drive.pose)
                                    .turnTo(Math.toRadians(270))
                                    .strafeToLinearHeading(new Vector2d(-34, 18.5 * c), Math.toRadians(270))
                                    .build()
                    );
                    robot.claw.openNext();
                    Actions.runBlocking(
                            robot.drive.actionBuilder(robot.drive.pose)
                                    .lineToY(12 * c)
                                    .build()
                    );
                    break;
                case RIGHT:
                    Actions.runBlocking(
                            robot.drive.actionBuilder(robot.drive.pose)
                                    .strafeToLinearHeading(new Vector2d(-34, 48 * c), Math.toRadians(outRot))
                                    .strafeToLinearHeading(new Vector2d(-34, 43 * c), Math.toRadians(outRot))
                                    .turn(Math.toRadians(-60))
                                    .lineToX(-29)
                                    .build()
                    );
                    robot.claw.openNext();
                    Actions.runBlocking(
                            robot.drive.actionBuilder(robot.drive.pose)
                                    .lineToX(-37)
                                    .turn(Math.toRadians(60))
                                    .strafeToLinearHeading(new Vector2d(-36, 43 * c), Math.toRadians(outRot))
                                    .strafeToLinearHeading(new Vector2d(-36, CrossFieldY * c), Math.toRadians(outRot))
                                    .build()
                    );
                    break;
            }
        }

        robot.vision.startProcessor(VisionProcessor.APRIL_TAG_DETECTION);

        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose)
//                        .strafeToLinearHeading(new Vector2d(-36, 36), Math.toRadians(90))
//                        .strafeToLinearHeading(new Vector2d(-36, CrossFieldY), Math.toRadians(180))
//                        .lineToY(CrossFieldY)
//                        .turnTo(Math.toRadians(180))
//                        .turn(Math.toRadians(-90))
                        .strafeToLinearHeading(new Vector2d(-36, CrossFieldY * c), Math.toRadians(180))
                        .build()
        );

//        robot.vision.startProcessor(VisionProcessor.APRIL_TAG_DETECTION);
        robot.vision.setActiveCamera(VisionProcessor.APRIL_TAG_DETECTION);

        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose)
                        .strafeToLinearHeading(new Vector2d(-35.9, 12 * c), Math.toRadians(180))
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
                        .strafeToLinearHeading(new Vector2d(0, CrossFieldY * c), Math.toRadians(180))
                        .strafeToLinearHeading(new Vector2d(PrepDeliverX, CrossFieldY * c), Math.toRadians(180))
                        .strafeToLinearHeading(new Vector2d(PrepDeliverX, boardDeliverY * c), Math.toRadians(180))
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
                        .strafeToLinearHeading(new Vector2d(DeliverX, prepBoardDeliverY * c), Math.toRadians(180))
                        .build()
        );

        robot.arm.setRotation(ArmRotation.LowDeliver);
        robot.arm.setGlobalWristRotation(true);
        robot.arm.update();

        Actions.runBlocking(
                new SleepAction(2)
        );

        ElapsedTime time = new ElapsedTime();

        boolean aligned = false;
        while (opModeIsActive()) {
            if (time.seconds() > AlignWithBoardTime) break;
            if (robot.vision.isBoardDetected(boardPosition)) {
                Optional<AprilTagPoseFtc> tpose = robot.vision.getBoardPose(boardPosition);
                if (tpose.isPresent()) {
                    Double margin = 0.0;
//                    if (boardPosition == BoardPosition.RIGHT) {
//                        margin = RightBonus;
//                    } else if (boardPosition == BoardPosition.LEFT) {
//                        margin = LeftBonus;
//                    }

                    Vector2d newPose = new Vector2d(robot.drive.pose.position.x + (tpose.get().y - BoardAlignmentConstants.DistFromBoard), robot.drive.pose.position.y - (tpose.get().x + margin));
                    Rotation2d heading = robot.drive.pose.heading.plus(Math.toRadians(tpose.get().yaw));

                    Actions.runBlocking(
                            robot.drive.actionBuilder(robot.drive.pose)
                                    .strafeToLinearHeading(newPose, heading)
                                    .build());

                    aligned = true;
                }
            }
        }
//
//        Actions.runBlocking(
//                robot.drive.actionBuilder(robot.drive.pose)
//                        .strafeToLinearHeading(new Vector2d(DeliverX, boardDeliverY), Math.toRadians(180))
//                        .build()
//        );
//
//        Actions.runBlocking(
//                robot.drive.actionBuilder(robot.drive.pose)
//                        .strafeToLinearHeading(new Vector2d(ExtraDeliverX, boardDeliverY), Math.toRadians(180))
//                        .build()
//        );


//                new ParallelAction(
//                        telemetryPacket -> {
//                            robot.arm.update();
//                            return false;
//                        }
//                false)

        robot.arm.update();

        robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-ChassisSpeed.BoardAlignmentSpeed, 0), 0));

        Actions.runBlocking(
                new SleepAction(PushBoardTime)
        );

        robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

        robot.claw.open();

        Actions.runBlocking(
                new SleepAction(1)
        );

        robot.arm.setRotation(ArmRotation.Down);
        robot.arm.setWristRotation(WristRotation.Down);

        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose)
                        .setReversed(true)
//                        .strafeToLinearHeading(new Vector2d(robot.drive.pose.position.x, ParkPositionPos.y), Math.toRadians(180))
                        .strafeToLinearHeading(new Vector2d(ParkPositionPos.x, robot.drive.pose.position.y), Math.toRadians(180))
                        .build()
        );

        while (opModeIsActive()) {
            idle();
        }
    }
}

