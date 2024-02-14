package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.auto.AutoColor;
import org.firstinspires.ftc.teamcode.util.auto.AutoSide;
import org.firstinspires.ftc.teamcode.util.auto.BoardPosition;
import org.firstinspires.ftc.teamcode.util.auto.RobotStorage;
import org.firstinspires.ftc.teamcode.util.auto.constants.BoardAlignmentConstants;
import org.firstinspires.ftc.teamcode.util.robot.AutoRobot;
import org.firstinspires.ftc.teamcode.util.robot.arm.ArmRotation;
import org.firstinspires.ftc.teamcode.util.robot.arm.ArmSpeed;
import org.firstinspires.ftc.teamcode.util.robot.arm.WristRotation;
import org.firstinspires.ftc.teamcode.util.robot.chassis.ChassisSpeed;
import org.firstinspires.ftc.teamcode.util.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.lang.Math;
import java.util.Optional;
import java.util.Timer;

@Config
public abstract class OldBaseAuto extends LinearOpMode {

    private AutoRobot robot;

    public static boolean FakeFail = false;

    public static boolean SuperAuto = false;
    public static boolean FastAuto = false;

    private static final Boolean Debug = true;

    protected static AutoSide SIDE = AutoSide.FAR;
    protected static AutoColor COLOR = AutoColor.RED;

    protected abstract void setConstants();

    public static double CrossFieldY = 9.0;

    // Right, Center, Left on blue side
    public static double[] AlignmentOffsetsBLUE = new double[]{-5.5, 6.0, 6.0};
    // Left Center Right on red side
    public static double[] AlignmentOffsetsRED = new double[]{5.5, -5.0, -5.0};

    public static double[] PrepDeliverY = new double[]{37.0, 35.0, 32.0};
    public static double[] DeliverY = new double[]{40.0, 33.0, 31.0};

    public static double LeftBonus = 1.0;
    public static double RightBonus = 1.0;
    public static double PrepDeliverX = 39.0;
    public static double CrossFieldX = 42.0;
    public static double DeliverX = 38.0;

    public static Vector2d ParkPositionPos = new Vector2d(56, 36);

    public static double ParkX = 62;

    public static double AlignWithBoardTime = 3.0;
    public static double PushBoardTime = 0.5;

    public static double StackY = 4;
    public static double StackX = -68;
    public static double PrepStackX = -58;

    BoardPosition boardPosition = BoardPosition.CENTER;

    @Override
    public void runOpMode() {
        setConstants();

        Pose2d startPose = RobotStorage.getStartPose(SIDE, COLOR);
        RobotStorage.setPose(startPose);
        robot = new AutoRobot(hardwareMap, telemetry, RobotStorage.pose);
        if (FastAuto) {
            robot.drive.defaultVelConstraint = robot.drive.fastVelConstraint;
            robot.drive.defaultAccelConstraint = robot.drive.fastAccelConstraint;
            robot.drive.defaultTurnConstraints = robot.drive.fastTurnConstraints;
        }

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

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Color: " + COLOR + " Side: " + SIDE);
            telemetry.addData(
                    "Status",
                    "Initialized for " + Math.round(timer.seconds())
            );
            boardPosition = robot.vision.getSpikeLocation();
            telemetry.addData("Spike Location", boardPosition.toString());
            telemetry.addData("Super Auto", SuperAuto);
            telemetry.update();
        }

        waitForStart();

        robot.vision.close();

        robot.claw.close();

        Actions.runBlocking(new SleepAction(0.5));

        t.schedule(
                new java.util.TimerTask() {
                    @Override
                    public void run() {
                        robot.arm.setRotation(ArmRotation.AutoHoldDown);
                    }
                },
                200
        );

        if (!SuperAuto) {
            ElapsedTime delayTimer = new ElapsedTime();
            if (FakeFail) {
                robot.drive.setDrivePowers(
                        new PoseVelocity2d(new Vector2d(0.06, 0), -0.08 * c)
                );

                if (boardPosition == BoardPosition.CENTER) {
                    while (delayTimer.seconds() < 9) {
                        robot.drive.updatePoseEstimate();
                    }
                } else {
                    while (delayTimer.seconds() < 8) {
                        robot.drive.updatePoseEstimate();
                    }
                }
            } else {
                if (boardPosition == BoardPosition.CENTER) {
                    while (delayTimer.seconds() < 9) {
                        idle();
                    }
                } else {
                    while (delayTimer.seconds() < 8) {
                        idle();
                    }
                }
            }
        }

        Actions.runBlocking(
                robot.drive
                        .actionBuilder(robot.drive.pose)
                        .strafeToLinearHeading(
                                new Vector2d(-36, 60 * c),
                                Math.toRadians(outRot)
                        )
                        .build()
        );

        switch (boardPosition) {
            case LEFT:
                double leftX = -36 + (c == 1 ? 5 : -6);
                Actions.runBlocking(
                        robot.drive
                                .actionBuilder(robot.drive.pose)
                                .strafeToLinearHeading(
                                        new Vector2d(-36, 48 * c),
                                        Math.toRadians(outRot)
                                )
                                .splineToSplineHeading(
                                        new Pose2d(leftX, 38 * c, Math.toRadians(outRot + 45)),
                                        Math.toRadians(outRot + 45)
                                )
                                .build()
                );
                Actions.runBlocking(
                        robot.drive
                                .actionBuilder(robot.drive.pose)
                                .strafeToLinearHeading(
                                        new Vector2d(leftX, 38 * c),
                                        Math.toRadians(outRot + 45),
                                        robot.drive.slowVelConstraint,
                                        robot.drive.slowAccelConstraint
                                )
                                .build()
                );
                robot.claw.openNext();
                Actions.runBlocking(
                        robot.drive
                                .actionBuilder(robot.drive.pose)
                                .strafeToLinearHeading(
                                        new Vector2d(c == 1 ? -40 : -36, 48 * c),
                                        Math.toRadians(outRot + 45)
                                )
                                //                                .lineToY(48 * c)
                                .turn(Math.toRadians(-45))
                                //                                .strafeToLinearHeading(new Vector2d(-36, CrossFieldY * c), Math.toRadians(outRot))
                                .build()
                );
                break;
            case CENTER:
                Actions.runBlocking(
                        robot.drive
                                .actionBuilder(robot.drive.pose)
                                .strafeToLinearHeading(
                                        new Vector2d(-36, 13 * c),
                                        Math.toRadians(outRot - 180)
                                )
                                .build()
                );
                Actions.runBlocking(
                        robot.drive
                                .actionBuilder(robot.drive.pose)
                                .strafeToLinearHeading(
                                        new Vector2d(-36, 13 * c),
                                        Math.toRadians(outRot - 180),
                                        robot.drive.slowVelConstraint,
                                        robot.drive.slowAccelConstraint
                                )
                                .build()
                );
                robot.claw.openNext();
                Actions.runBlocking(
                        robot.drive
                                .actionBuilder(robot.drive.pose)
                                .lineToY(9 * c)
                                .strafeToLinearHeading(
                                        new Vector2d(-30, CrossFieldY * c),
                                        Math.toRadians(180)
                                )
                                .build()
                );
                break;
            case RIGHT:
                double rightX = -36 - (c == 1 ? 6 : -6);
                Actions.runBlocking(
                        robot.drive
                                .actionBuilder(robot.drive.pose)
                                .strafeToLinearHeading(
                                        new Vector2d(-36, 48 * c),
                                        Math.toRadians(outRot)
                                )
                                .splineToSplineHeading(
                                        new Pose2d(rightX, 38 * c, Math.toRadians(outRot - 45)),
                                        Math.toRadians(outRot + 45)
                                )
                                .build()
                );
                Actions.runBlocking(
                        robot.drive
                                .actionBuilder(robot.drive.pose)
                                .strafeToLinearHeading(
                                        new Vector2d(rightX, 38 * c),
                                        Math.toRadians(outRot - 45),
                                        robot.drive.slowVelConstraint,
                                        robot.drive.slowAccelConstraint
                                )
                                .build()
                );
                robot.claw.openNext();
                Actions.runBlocking(
                        robot.drive
                                .actionBuilder(robot.drive.pose)
                                .strafeToLinearHeading(
                                        new Vector2d(c == 1 ? -36 : -40, 48 * c),
                                        Math.toRadians(outRot - 45)
                                )
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

        double boardDeliverY = DeliverY[1];
        double prepBoardDeliverY = PrepDeliverY[1];

        BoardPosition targetTag = BoardPosition.CENTER;
        double alignmentOffset = COLOR == AutoColor.BLUE
                ? AlignmentOffsetsBLUE[1]
                : AlignmentOffsetsRED[1];

        if (boardPosition == BoardPosition.CENTER) {
            if (COLOR == AutoColor.BLUE) {
                targetTag = BoardPosition.RIGHT;
            } else {
                targetTag = BoardPosition.LEFT;
            }
        } else {
            if (COLOR == AutoColor.BLUE) {
                if (boardPosition == BoardPosition.LEFT) {
                    alignmentOffset = AlignmentOffsetsBLUE[2];
                } else {
                    alignmentOffset = AlignmentOffsetsBLUE[0];
                }
            } else {
                if (boardPosition == BoardPosition.LEFT) {
                    alignmentOffset = AlignmentOffsetsRED[0];
                } else {
                    alignmentOffset = AlignmentOffsetsRED[2];
                }
            }
        }

        switch (targetTag) {
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
                    robot.drive
                            .actionBuilder(robot.drive.pose)
                            .strafeToLinearHeading(
                                    new Vector2d(PrepDeliverX, CrossFieldY * c),
                                    Math.toRadians(180)
                            )
                            //                            .splineToConstantHeading(new Vector2d(PrepDeliverX, boardDeliverY * c), Math.toRadians(180))
                            .strafeToLinearHeading(
                                    new Vector2d(PrepDeliverX, boardDeliverY * c),
                                    Math.toRadians(180)
                            )
                            .build()
            );
        } else {
            Actions.runBlocking(
                    robot.drive
                            .actionBuilder(robot.drive.pose)
                            .splineTo(new Vector2d(-36, 20 * c), Math.toRadians(outRot))
                            .splineTo(new Vector2d(-24, CrossFieldY * c), Math.toRadians(0))
                            //                            .splineTo(new Vector2d(12, CrossFieldY * c), Math.toRadians(0))
                            .strafeToLinearHeading(
                                    new Vector2d(CrossFieldX, CrossFieldY * c),
                                    Math.toRadians(0)
                            )
                            //                            .splineToLinearHeading(new Pose2d(PrepDeliverX, boardDeliverY * c, Math.toRadians(180)), Math.toRadians(180))
                            .strafeToLinearHeading(
                                    new Vector2d(PrepDeliverX, boardDeliverY * c),
                                    Math.toRadians(180)
                            )
                            .build()
            );
        }

        robot.arm.setRotation(ArmRotation.AutoDeliver);
        robot.arm.setGlobalWristRotation(true);

        Actions.runBlocking(
                robot.drive
                        .actionBuilder(robot.drive.pose)
                        .strafeToLinearHeading(
                                new Vector2d(PrepDeliverX, boardDeliverY * c),
                                Math.toRadians(180),
                                robot.drive.slowVelConstraint,
                                robot.drive.slowAccelConstraint
                        )
                        .build()
        );

        robot.arm.update();

        Actions.runBlocking(new SleepAction(1));

        robot.arm.setRotation(ArmRotation.AutoDeliverLow, ArmSpeed.Min);
        robot.arm.update();

        Actions.runBlocking(new SleepAction(1));

        robot.arm.setRotation(ArmRotation.AutoDeliverLow, ArmSpeed.Max);

        ElapsedTime time = new ElapsedTime();

        boolean aligned = false;
        while (opModeIsActive()) {
            if (time.seconds() > AlignWithBoardTime) break;
            robot.arm.update();
            if (robot.vision.isBoardDetected(targetTag)) {
                Optional<AprilTagPoseFtc> tpose = robot.vision.getBoardPose(targetTag);
                if (tpose.isPresent()) {
                    double margin = 0.0;

                    Vector2d newPose = new Vector2d(
                            robot.drive.pose.position.x +
                                    (tpose.get().y - (BoardAlignmentConstants.DistFromBoard + 1)),
                            robot.drive.pose.position.y - (tpose.get().x + margin)
                    );
                    Rotation2d heading = robot.drive.pose.heading.plus(
                            Math.toRadians(tpose.get().yaw)
                    );

                    aligned = true;

                    Actions.runBlocking(
                            robot.drive
                                    .actionBuilder(robot.drive.pose)
                                    .strafeToLinearHeading(
                                            newPose,
                                            heading,
                                            robot.drive.slowVelConstraint,
                                            robot.drive.slowAccelConstraint
                                    )
                                    .build()
                    );
                }
            }
        }

        if (!aligned) {
            telemetry.addLine("ERROR: APRIL TAG CAMERA NOT WORKING!!!");
            telemetry.update();

            robot.drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(-ChassisSpeed.BoardAlignmentSpeed, 0),
                            0
                    )
            );

            Actions.runBlocking(new SleepAction(PushBoardTime));

            robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
        }

        robot.arm.update();

        Actions.runBlocking(
                robot.drive
                        .actionBuilder(robot.drive.pose)
                        .strafeTo(
                                new Vector2d(
                                        robot.drive.pose.position.x,
                                        robot.drive.pose.position.y + alignmentOffset
                                ),
                                robot.drive.slowVelConstraint,
                                robot.drive.slowAccelConstraint
                        )
                        .build()
        );

        //        robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-ChassisSpeed.BoardAlignmentSpeed, 0), 0));

        //        Actions.runBlocking(
        //                new SleepAction(PushBoardTime)
        //        );

        //        robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
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

        Actions.runBlocking(new SleepAction(0.5));

        robot.arm.setRotation(ArmRotation.AutoDeliver);

        Actions.runBlocking(new SleepAction(0.2));
        //
        //        robot.arm.setRotation(ArmRotation.AutoDeliverLow);
        //
        //        Actions.runBlocking(
        //                new SleepAction(0.5)
        //        );

        robot.arm.setRotation(ArmRotation.Down);
        robot.arm.setWristRotation(WristRotation.Down);

        if (!SuperAuto) {
            Actions.runBlocking(
                    robot.drive
                            .actionBuilder(robot.drive.pose)
                            //                        .strafeToLinearHeading(new Vector2d(robot.drive.pose.position.x, ParkPositionPos.y), Math.toRadians(180))
                            //                        .strafeToLinearHeading(new Vector2d(ParkPositionPos.x, robot.drive.pose.position.y), Math.toRadians(180))
                            .lineToX(ParkX)
                            .build()
            );
        } else {
            Actions.runBlocking(
                    robot.drive
                            .actionBuilder(robot.drive.pose)
                            .splineToLinearHeading(
                                    new Pose2d(42, CrossFieldY * c, Math.toRadians(180)),
                                    Math.toRadians(0)
                            )
                            .strafeToLinearHeading(
                                    new Vector2d(-59, CrossFieldY * c),
                                    Math.toRadians(180)
                            )
                            .build()
            );

            robot.claw.open();
            robot.arm.setRotation(ArmRotation.Stack4);

            Actions.runBlocking(
                    robot.drive
                            .actionBuilder(robot.drive.pose)
                            .strafeToLinearHeading(
                                    new Vector2d(PrepStackX, StackY * c),
                                    Math.toRadians(180),
                                    robot.drive.slowVelConstraint,
                                    robot.drive.slowAccelConstraint
                            )
                            .strafeToLinearHeading(
                                    new Vector2d(StackX, StackY * c),
                                    Math.toRadians(180),
                                    robot.drive.slowVelConstraint,
                                    robot.drive.slowAccelConstraint
                            )
                            .build()
            );

            robot.arm.setRotation(ArmRotation.Stack3);

            robot.claw.close();

            Actions.runBlocking(new SleepAction(0.5));

            robot.claw.close();

            Actions.runBlocking(
                    robot.drive
                            .actionBuilder(robot.drive.pose)
                            .splineToLinearHeading(
                                    new Pose2d(42, CrossFieldY * c, Math.toRadians(180)),
                                    Math.toRadians(180)
                            )
                            .splineTo(new Vector2d(60, CrossFieldY * c), Math.toRadians(90))
                            .build()
            );

            robot.claw.open();

            Actions.runBlocking(
                    robot.drive
                            .actionBuilder(robot.drive.pose)
                            .strafeTo(new Vector2d(62, 7 * c))
                            .build()
            );
        }

        robot.vision.close();

        telemetry.addLine(
                "Auto Finished in " + Math.round(timer.seconds()) + " seconds"
        );
        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }
    }
}
