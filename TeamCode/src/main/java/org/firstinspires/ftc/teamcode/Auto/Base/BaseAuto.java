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

    public static boolean FakeFail = true;

    private final static Boolean Debug = true;

    protected static AutoSide SIDE = AutoSide.FAR;
    protected static AutoColor COLOR = AutoColor.RED;

    protected abstract void setConstants();

    public static Double CrossFieldY = 9.0;

    public static Double[] PrepDeliverY = new Double[]{37.0, 33.0, 29.0};
    public static Double[] DeliverY = new Double[]{40.0, 33.0, 28.0};
    public static Double LeftBonus = 1.0;
    public static Double RightBonus = 1.0;
    public static Double PrepDeliverX = 39.0;
    public static Double DeliverX = 38.0;

    public static Vector2d ParkPositionPos = new Vector2d(56, 36);

    public static Double AlignWithBoardTime = 2.0;
    public static Double PushBoardTime = 0.5;

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

        robot.claw.close();

        Actions.runBlocking(
                new SleepAction(0.5)
        );


        t.schedule(
                new java.util.TimerTask() {
                    @Override
                    public void run() {
                        robot.arm.setRotation(ArmRotation.AutoHoldDown);
                    }
                },
                200
        );

        if (FakeFail) {
            robot.drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(
                                    0.06,
                                    0
                            ),
                            -0.08 * c
                    )
            );

            ElapsedTime fakeBadAuto = new ElapsedTime();

            if (boardPosition == BoardPosition.CENTER) {
                while (fakeBadAuto.seconds() < 7) {
                    robot.drive.updatePoseEstimate();
                }
            } else {
                while (fakeBadAuto.seconds() < 6) {
                    robot.drive.updatePoseEstimate();
                }
            }
        }


        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose)
                        .strafeToLinearHeading(new Vector2d(-36, 60 * c), Math.toRadians(outRot))
                        .build()
        );

        switch (boardPosition) {
            case LEFT:
                double leftX = -36 + (c == 1 ? 4 : -6);
                Actions.runBlocking(
                        robot.drive.actionBuilder(robot.drive.pose)
                                .strafeToLinearHeading(new Vector2d(-36, 48 * c), Math.toRadians(outRot))
                                .splineToSplineHeading(new Pose2d(leftX, 38 * c, Math.toRadians(outRot + 45)), Math.toRadians(outRot + 45))
                                .build()
                );
                Actions.runBlocking(
                        robot.drive.actionBuilder(robot.drive.pose)
                                .strafeToLinearHeading(new Vector2d(leftX, 38 * c), Math.toRadians(outRot + 45), robot.drive.slowVelConstraint, robot.drive.slowAccelConstraint)
                                .build()
                );
                robot.claw.openNext();
                Actions.runBlocking(
                        robot.drive.actionBuilder(robot.drive.pose)
                                .strafeToLinearHeading(new Vector2d(c == 1 ? -40 : -36, 48 * c), Math.toRadians(outRot + 45))
//                                .lineToY(48 * c)
                                .turn(Math.toRadians(-45))
//                                .strafeToLinearHeading(new Vector2d(-36, CrossFieldY * c), Math.toRadians(outRot))
                                .build()
                );
                break;
            case CENTER:
                Actions.runBlocking(
                        robot.drive.actionBuilder(robot.drive.pose)
                                .strafeToLinearHeading(new Vector2d(-36, 13 * c), Math.toRadians(outRot - 180))
                                .build()
                );
                Actions.runBlocking(
                        robot.drive.actionBuilder(robot.drive.pose)
                                .strafeToLinearHeading(new Vector2d(-36, 13 * c), Math.toRadians(outRot - 180), robot.drive.slowVelConstraint, robot.drive.slowAccelConstraint)
                                .build()
                );
                robot.claw.openNext();
                Actions.runBlocking(
                        robot.drive.actionBuilder(robot.drive.pose)
                                .lineToY(9 * c)
                                .strafeToLinearHeading(new Vector2d(-30, CrossFieldY * c), Math.toRadians(180))
                                .build()
                );
                break;
            case RIGHT:
                double rightX = -36 - (c == 1 ? 6 : -6);
                Actions.runBlocking(
                        robot.drive.actionBuilder(robot.drive.pose)
                                .strafeToLinearHeading(new Vector2d(-36, 48 * c), Math.toRadians(outRot))
                                .splineToSplineHeading(new Pose2d(rightX, 38 * c, Math.toRadians(outRot - 45)), Math.toRadians(outRot + 45))
                                .build()
                );
                Actions.runBlocking(
                        robot.drive.actionBuilder(robot.drive.pose)
                                .strafeToLinearHeading(new Vector2d(rightX, 38 * c), Math.toRadians(outRot - 45), robot.drive.slowVelConstraint, robot.drive.slowAccelConstraint)
                                .build()
                );
                robot.claw.openNext();
                Actions.runBlocking(
                        robot.drive.actionBuilder(robot.drive.pose)
                                .strafeToLinearHeading(new Vector2d(c == 1 ? -36 : -40, 48 * c), Math.toRadians(outRot - 45))
//                                .lineToY(48 * c)
                                .turn(Math.toRadians(45))
//                                .strafeToLinearHeading(new Vector2d(-36, CrossFieldY * c), Math.toRadians(outRot))
                                .build()
                );
                break;
        }

        robot.vision.startProcessor(VisionProcessor.APRIL_TAG_DETECTION);

//        Actions.runBlocking(
//                robot.drive.actionBuilder(robot.drive.pose)
//                        .strafeToLinearHeading(new Vector2d(-35.9, CrossFieldY * c), Math.toRadians(180))
//                        .build()
//        );

        robot.vision.setActiveCamera(VisionProcessor.APRIL_TAG_DETECTION);

        Double boardDeliverY = DeliverY[1];
        Double prepBoardDeliverY = PrepDeliverY[1];

        switch (boardPosition) {
            case LEFT:
                if (COLOR == AutoColor.BLUE) {
                    boardDeliverY = DeliverY[0];
                    prepBoardDeliverY = PrepDeliverY[0];
                } else {
                    boardDeliverY = DeliverY[2];
                    prepBoardDeliverY = PrepDeliverY[2];
                }
                break;
            case RIGHT:
                if (COLOR == AutoColor.BLUE) {
                    boardDeliverY = DeliverY[2];
                    prepBoardDeliverY = PrepDeliverY[2];
                } else {
                    boardDeliverY = DeliverY[0];
                    prepBoardDeliverY = PrepDeliverY[0];
                }
                break;
        }

        if (boardPosition == BoardPosition.CENTER) {
            Actions.runBlocking(
                    robot.drive.actionBuilder(robot.drive.pose)
                            .strafeToLinearHeading(new Vector2d(PrepDeliverX, CrossFieldY * c), Math.toRadians(180))
                            .splineToConstantHeading(new Vector2d(PrepDeliverX, boardDeliverY * c), Math.toRadians(180))
                            .build()
            );
        } else {
            Actions.runBlocking(
                    robot.drive.actionBuilder(robot.drive.pose)
                            .splineToLinearHeading(new Pose2d(-36, 24 * c, Math.toRadians(outRot)), Math.toRadians(outRot))
                            .splineToLinearHeading(new Pose2d(-24, CrossFieldY * c, Math.toRadians(0)), Math.toRadians(0))
                            .strafeToLinearHeading(new Vector2d(PrepDeliverX, CrossFieldY * c), Math.toRadians(0))
                            .splineToLinearHeading(new Pose2d(PrepDeliverX, boardDeliverY * c, Math.toRadians(180)), Math.toRadians(180))
                            .build()
            );
        }

        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose)
                        .strafeToLinearHeading(new Vector2d(PrepDeliverX, boardDeliverY * c), Math.toRadians(180), robot.drive.slowVelConstraint, robot.drive.slowAccelConstraint)
                        .build()
        );

        robot.arm.setRotation(ArmRotation.AutoDeliver);
        robot.arm.setGlobalWristRotation(true);
        robot.arm.update();

        Actions.runBlocking(
                new SleepAction(1)
        );

        ElapsedTime time = new ElapsedTime();

        boolean aligned = false;
        while (opModeIsActive()) {
            if (time.seconds() > AlignWithBoardTime) break;
            if (robot.vision.isBoardDetected(boardPosition)) {
                Optional<AprilTagPoseFtc> tpose = robot.vision.getBoardPose(boardPosition);
                if (tpose.isPresent()) {
                    double margin = 0.0;
//                    if (boardPosition == BoardPosition.RIGHT) {
//                        margin = RightBonus;
//                    } else if (boardPosition == BoardPosition.LEFT) {
//                        margin = LeftBonus;
//                    }

                    Vector2d newPose = new Vector2d(robot.drive.pose.position.x + (tpose.get().y - (BoardAlignmentConstants.DistFromBoard + 1)), robot.drive.pose.position.y - (tpose.get().x + margin));
                    Rotation2d heading = robot.drive.pose.heading.plus(Math.toRadians(tpose.get().yaw));

                    Actions.runBlocking(
                            robot.drive.actionBuilder(robot.drive.pose)
                                    .strafeToLinearHeading(newPose, heading, robot.drive.slowVelConstraint, robot.drive.slowAccelConstraint)
                                    .build()
                    );

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
//
//        Actions.runBlocking(
//                robot.drive.actionBuilder(robot.drive.pose)
//                        .strafeTo(new Vector2d(robot.drive.pose.position.x, robot.drive.pose.position.y - 5))
//                        .build()
//        );
//
//        robot.arm.setRotation(ArmRotation.AutoDeliverLow);
//
//        Actions.runBlocking(
//                robot.drive.actionBuilder(robot.drive.pose)
//                        .strafeTo(new Vector2d(robot.drive.pose.position.x, robot.drive.pose.position.y + 8))
//                        .build()
//        );

        robot.claw.open();


        Actions.runBlocking(
                new SleepAction(0.5)
        );

        robot.arm.setRotation(ArmRotation.AutoDeliver - 10);

        Actions.runBlocking(
                new SleepAction(1)
        );

        robot.arm.setRotation(ArmRotation.AutoDeliver);

        Actions.runBlocking(
                new SleepAction(0.5)
        );

        robot.arm.setRotation(ArmRotation.Down);
        robot.arm.setWristRotation(WristRotation.Down);

        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose)
//                        .strafeToLinearHeading(new Vector2d(robot.drive.pose.position.x, ParkPositionPos.y), Math.toRadians(180))
//                        .strafeToLinearHeading(new Vector2d(ParkPositionPos.x, robot.drive.pose.position.y), Math.toRadians(180))
                        .lineToX(ParkPositionPos.x)
                        .build()
        );

        while (opModeIsActive()) {
            idle();
        }
    }
}

