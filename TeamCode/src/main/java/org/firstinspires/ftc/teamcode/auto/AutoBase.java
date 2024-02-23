package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.roadrunner.speed.TrajectorySpeed;
import org.firstinspires.ftc.teamcode.util.auto.AutoColor;
import org.firstinspires.ftc.teamcode.util.auto.AutoSide;
import org.firstinspires.ftc.teamcode.util.auto.BoardPosition;
import org.firstinspires.ftc.teamcode.util.auto.RobotStorage;
import org.firstinspires.ftc.teamcode.util.auto.constants.BoardAlignmentConstants;
import org.firstinspires.ftc.teamcode.util.robot.AutoRobot;
import org.firstinspires.ftc.teamcode.util.robot.arm.ArmRotation;
import org.firstinspires.ftc.teamcode.util.robot.arm.ArmSpeed;
import org.firstinspires.ftc.teamcode.util.robot.arm.WristRotation;
import org.firstinspires.ftc.teamcode.util.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.lang.Math;
import java.util.Optional;
import java.util.Timer;

@Config
public abstract class AutoBase extends LinearOpMode {
    private final static Boolean Debug = true;

    private AutoRobot robot;

    protected AutoSide SIDE = AutoSide.FAR;
    protected AutoColor COLOR = AutoColor.RED;

    protected abstract AutoSide getSide();

    protected abstract AutoColor getColor();

    BoardPosition boardPosition = BoardPosition.CENTER;

    public static double FarLaneY = 14;

    // Right, Center, Left on blue side
    public static double[] AlignmentOffsetsBLUE = new double[]{-5.5, 6.0, 6.7};
    // Left Center Right on red side
    public static double[] AlignmentOffsetsRED = new double[]{5.5, -5.0, -4.5};

    public static double WhiteAlignWithBoardTime = 1.5;
    public static double AlignWithBoardTime = 2.5;
    public static double PushBoardTime = 0.5;

    public static double ParkX = 58;

    public static Vector2d stack1 = new Vector2d(-50, 39);
    public static Vector2d blueStack = new Vector2d(-50, 39);
    public static Vector2d redStack = new Vector2d(-50, 36);

    int c;
    double startHeading;

    @Override
    public void runOpMode() {
        SIDE = getSide();
        COLOR = getColor();

        Pose2d startPose = RobotStorage.getStartPose(SIDE, COLOR);
        RobotStorage.setPose(startPose);
        robot = new AutoRobot(hardwareMap, startPose);

        robot.vision.startProcessor(VisionProcessor.SPIKE_LOCATION_DETECTION);
        robot.vision.setColor(COLOR);

        ElapsedTime elapsedTime = new ElapsedTime();
        Timer t = new Timer();

        c = COLOR.value;
        startHeading = COLOR == AutoColor.BLUE ? 270 : 90;
        stack1 = COLOR == AutoColor.BLUE ? blueStack : redStack;

        while ((!isStarted() || !robot.vision.isReady()) && !isStopRequested()) {
            telemetry.addLine("Color: " + COLOR + " Side: " + SIDE);
            telemetry.addData("Status", "Initialized for " + Math.round(elapsedTime.seconds()));
            telemetry.addLine("---");

            boardPosition = robot.vision.getSpikeLocation();
            telemetry.addData("Spike Location", boardPosition.toString());
            telemetry.update();
        }

        waitForStart();

        if (isStopRequested()) return;

        elapsedTime.reset();

        robot.vision.close();

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

        if (SIDE == AutoSide.FAR) {
            deliverSpikeMarkFar();
        } else {
            deliverSpikeMarkNear();
        }

        // Pickup from white stack

        robot.arm.setRotation(ArmRotation.StackAuto);
        robot.arm.setWristRotation(WristRotation.StackDown);

        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose, TrajectorySpeed.SLOW)
                        .strafeToLinearHeading(new Vector2d(-62, stack1.y * c), Math.toRadians(180))
                        .build()
        );

        robot.drive.setDrivePowersForSeconds(new PoseVelocity2d(
                new Vector2d(0.35, 0), 0
        ), 1);

        robot.drive.pose = new Pose2d(-65, robot.drive.pose.position.y, Math.toRadians(180));

        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose, TrajectorySpeed.SLOW)
                        .strafeToLinearHeading(new Vector2d(-63.5, stack1.y * c), Math.toRadians(180))
                        .build()
        );

        robot.arm.setRotation(ArmRotation.StackDownAuto);
        robot.arm.setWristRotation(WristRotation.AutoPickupStack);

        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose, TrajectorySpeed.SLOW)
                        .strafeToLinearHeading(new Vector2d(-65, stack1.y * c), Math.toRadians(180))
                        .build()
        );

        Actions.runBlocking(new SleepAction(0.2));

        robot.claw.close();

        Actions.runBlocking(new SleepAction(0.5));

        robot.arm.setRotation(ArmRotation.HoldDown);
        robot.arm.setWristRotation(WristRotation.Down);

        // Switch to April Tag detection

        robot.vision.startProcessor(VisionProcessor.APRIL_TAG_DETECTION);

        robot.vision.setActiveCamera(VisionProcessor.APRIL_TAG_DETECTION);

        // Drive across field

        double strafeToFarLaneX = -50;
        if (COLOR == AutoColor.BLUE) {
            if (boardPosition == BoardPosition.LEFT) {
                strafeToFarLaneX = -42;
            } else if (boardPosition == BoardPosition.RIGHT) {
                strafeToFarLaneX = -34;
            }
        } else {
            if (boardPosition == BoardPosition.LEFT) {
                strafeToFarLaneX = -34;
            } else if (boardPosition == BoardPosition.RIGHT) {
                strafeToFarLaneX = -42;
            }
        }

        Actions.runBlocking(
                new SequentialAction(
                        robot.drive.actionBuilder(robot.drive.pose, TrajectorySpeed.FAST)
                                .strafeToLinearHeading(new Vector2d(strafeToFarLaneX, 36 * c), Math.toRadians(180))
                                .strafeToLinearHeading(new Vector2d(strafeToFarLaneX, FarLaneY * c), Math.toRadians(180))
                                .strafeToLinearHeading(new Vector2d(30, FarLaneY * c), Math.toRadians(180))
                                .build(),
                        robot.drive.actionBuilder(new Pose2d(30, FarLaneY * c, Math.toRadians(180)))
                                .strafeToLinearHeading(new Vector2d(30, 32 * c), Math.toRadians(180))
                                .build()
                )
        );

        robot.arm.setRotation(ArmRotation.MidDeliver);
        robot.arm.setGlobalWristRotation(true);
        robot.arm.update();


        // Delivery to board

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

        robot.claw.setFingerEnabled(true);

        robot.arm.setRotation(ArmRotation.PrepAutoDeliver);
        robot.arm.update();

        BoardPosition whiteTag = COLOR == AutoColor.BLUE ? BoardPosition.RIGHT : BoardPosition.LEFT;

        if (COLOR == AutoColor.BLUE) {
            if (boardPosition == BoardPosition.RIGHT) {
                whiteTag = BoardPosition.LEFT;
            }
        } else {
            if (boardPosition == BoardPosition.LEFT) {
                whiteTag = BoardPosition.RIGHT;
            }
        }

        boolean whiteAligned = alignWithAprilTag(whiteTag, WhiteAlignWithBoardTime);


        robot.arm.setRotation(ArmRotation.AutoDeliver, ArmSpeed.Mid);
        robot.arm.update();

        Actions.runBlocking(
                robot.drive
                        .actionBuilder(robot.drive.pose, TrajectorySpeed.SLOW)
                        .strafeTo(
                                new Vector2d(
                                        robot.drive.pose.position.x + 2,
                                        robot.drive.pose.position.y
                                )
                        )
                        .build()
        );

        robot.claw.openNext();

        Actions.runBlocking(new SleepAction(0.5));

        Actions.runBlocking(
                robot.drive
                        .actionBuilder(robot.drive.pose, TrajectorySpeed.SLOW)
                        .strafeTo(
                                new Vector2d(
                                        32,
                                        robot.drive.pose.position.y + (8 * c)
                                )
                        )
                        .build()
        );

        robot.arm.setRotation(ArmRotation.AutoDeliver);
        robot.arm.setBoardAngle(WristRotation.AutoBoardAngle);
//        robot.arm.setBoardAngle(WristRotation.AutoBoardAngle);
        robot.arm.update();

        boolean aligned = alignWithAprilTag(boardPosition, AlignWithBoardTime);
//
//        if (!aligned) {
//            telemetry.addLine("ERROR: APRIL TAG CAMERA NOT WORKING!!!");
//            telemetry.update();
//
//            Actions.runBlocking(
//                    robot.drive.actionBuilder(robot.drive.pose)
//                            .strafeToLinearHeading(new Vector2d(38, 28 * c), Math.toRadians(180))
//                            .build()
//            );
//
//            robot.claw.openNext();
//
//            Actions.runBlocking(
//                    robot.drive.actionBuilder(robot.drive.pose)
//                            .strafeToLinearHeading(new Vector2d(30, 34 * c), Math.toRadians(180))
//                            .build()
//            );
//
//            robot.drive.setDrivePowersForSeconds(
//                    new PoseVelocity2d(
//                            new Vector2d(-ChassisSpeed.BoardAlignmentSpeed, 0),
//                            0
//                    ),
//                    1
//            );
//        }

        robot.arm.update();

        final double alignedY = robot.drive.pose.position.y;

        Actions.runBlocking(
                robot.drive
                        .actionBuilder(robot.drive.pose, TrajectorySpeed.SLOW)
                        .strafeTo(
                                new Vector2d(
                                        robot.drive.pose.position.x + 1,
                                        alignedY
                                )
                        )
                        .build()
        );

//        Actions.runBlocking(
//                robot.drive
//                        .actionBuilder(robot.drive.pose, TrajectorySpeed.SLOW)
//                        .strafeTo(
//                                new Vector2d(
//                                        robot.drive.pose.position.x + 0.5,
//                                        alignedY + alignmentOffset
//                                )
//                        )
//                        .build()
//        );

        robot.claw.open();

        Actions.runBlocking(new SleepAction(0.5));

        robot.arm.setRotation(ArmRotation.AutoDeliver);

        Actions.runBlocking(new SleepAction(0.2));

        robot.claw.setFingerEnabled(false);
        robot.arm.setRotation(ArmRotation.Down);
        robot.arm.setWristRotation(WristRotation.Down);

        Actions.runBlocking(
                robot.drive
                        .actionBuilder(robot.drive.pose)
                        .strafeTo(
                                new Vector2d(
                                        robot.drive.pose.position.x + 12,
                                        robot.drive.pose.position.y
                                )
                        )
                        .build()
        );

        robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-0.3, 0), 0));

        telemetry.addLine("Auto Finished in " + Math.round(elapsedTime.seconds()) + " seconds");
        telemetry.update();

        ElapsedTime driveIntoBoardTimer = new ElapsedTime();

        while (driveIntoBoardTimer.seconds() < 2 && !isStopRequested()) {
            idle();
        }

        robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));

        while (opModeIsActive() && !isStopRequested()) {
            idle();
        }
    }

    private void deliverSpikeMarkFar() {
        switch (boardPosition) {
            case LEFT:
                double leftX = -36 + (COLOR == AutoColor.BLUE ? 7 : -6);
                Actions.runBlocking(
                        new SequentialAction(
                                robot.drive.actionBuilder(robot.drive.pose)
                                        .strafeToLinearHeading(
                                                new Vector2d(-36, 48 * c),
                                                Math.toRadians(startHeading)
                                        )
                                        .splineToSplineHeading(
                                                new Pose2d(leftX, 38 * c, Math.toRadians(startHeading + 45)),
                                                Math.toRadians(startHeading + 45)
                                        )
                                        .strafeToLinearHeading(
                                                new Vector2d(leftX, 38 * c),
                                                Math.toRadians(startHeading + 45),
                                                robot.drive.getSpeedConstraint(TrajectorySpeed.SLOW).velConstraint
                                        ).build(),
                                new SleepAction(0.2),
                                robot.openNextClaw(),
                                new SleepAction(0.2),
                                robot.drive.actionBuilder(new Pose2d(leftX, 38 * c, Math.toRadians(startHeading + 45)))
                                        .strafeToLinearHeading(
                                                new Vector2d(-36, 48 * c),
                                                Math.toRadians(startHeading + 45)
                                        )
                                        .build(),
                                robot.raiseArmForStack(),
                                robot.drive.actionBuilder(new Pose2d(-36, 48 * c, Math.toRadians(startHeading + 45)))
                                        .splineToLinearHeading(
                                                new Pose2d(stack1.x, stack1.y * c, Math.toRadians(180)),
                                                Math.toRadians(0)
                                        )
                                        .build()
                        )
                );
                break;
            case CENTER:
                Actions.runBlocking(
                        new SequentialAction(
                                robot.drive.actionBuilder(robot.drive.pose)
                                        .strafeToLinearHeading(
                                                new Vector2d(-36, 50 * c),
                                                Math.toRadians(startHeading)
                                        )
                                        .splineToSplineHeading(
                                                new Pose2d(33, 34 * c, Math.toRadians(startHeading)),
                                                Math.toRadians(startHeading)
                                        )
                                        .strafeToLinearHeading(
                                                new Vector2d(33, 32 * c),
                                                Math.toRadians(startHeading + 45),
                                                robot.drive.getSpeedConstraint(TrajectorySpeed.SLOW).velConstraint,
                                                robot.drive.getSpeedConstraint(TrajectorySpeed.SLOW).accelConstraint
                                        ).build(),
                                new SleepAction(0.2),
                                robot.openNextClaw(),
                                new SleepAction(0.2),
                                robot.drive.actionBuilder(new Pose2d(33, 32 * c, Math.toRadians(startHeading)))
                                        .strafeToLinearHeading(
                                                new Vector2d(-34, 48 * c),
                                                Math.toRadians(startHeading)
                                        )
                                        .build(),
                                robot.raiseArmForStack(),
                                robot.drive.actionBuilder(new Pose2d(-36, 48 * c, Math.toRadians(startHeading + 45)))
                                        .splineToLinearHeading(
                                                new Pose2d(stack1.x, stack1.y * c, Math.toRadians(180)),
                                                Math.toRadians(0)
                                        )
                                        .build()
                        )
                );
                break;
            case RIGHT:
                double rightX = -36 - (COLOR == AutoColor.BLUE ? 6 : -7);
                Actions.runBlocking(
                        new SequentialAction(
                                robot.drive.actionBuilder(robot.drive.pose)
                                        .strafeToLinearHeading(
                                                new Vector2d(c == 1 ? -36 : -38, 48 * c),
                                                Math.toRadians(startHeading)
                                        )
                                        .splineToSplineHeading(
                                                new Pose2d(rightX, 38 * c, Math.toRadians(startHeading - 45)),
                                                Math.toRadians(startHeading - 45)
                                        )
                                        .strafeToLinearHeading(
                                                new Vector2d(rightX, 38 * c),
                                                Math.toRadians(startHeading - 45),
                                                robot.drive.getSpeedConstraint(TrajectorySpeed.SLOW).velConstraint
                                        ).build(),
                                robot.openNextClaw(),
                                new SleepAction(0.3),
                                robot.drive.actionBuilder(new Pose2d(rightX, 38 * c, Math.toRadians(startHeading - 45)))
                                        .strafeToLinearHeading(
                                                new Vector2d(-36, 48 * c),
                                                Math.toRadians(startHeading - 45)
                                        )
                                        .build(),
                                robot.raiseArmForStack(),
                                robot.drive.actionBuilder(new Pose2d(-36, 48 * c, Math.toRadians(startHeading - 45)))
                                        .splineToLinearHeading(
                                                new Pose2d(stack1.x + 2, stack1.y * c, Math.toRadians(180)),
                                                Math.toRadians(0)
                                        )
                                        .build(),
                                robot.drive.actionBuilder(new Pose2d(stack1.x + 2, stack1.y * c, Math.toRadians(180)), TrajectorySpeed.SLOW)
                                        .strafeToLinearHeading(
                                                new Vector2d(stack1.x, stack1.y * c),
                                                Math.toRadians(180)
                                        )
                                        .build()
                        )
                );
                break;
        }
    }

    private void deliverSpikeMarkNear() {

    }

    public boolean alignWithAprilTag(BoardPosition targetTag, double timeout) {
        ElapsedTime time = new ElapsedTime();

        boolean aligned = false;
        while (opModeIsActive()) {
            if (time.seconds() > timeout) break;
            robot.arm.update();
            if (robot.vision.isBoardDetected(targetTag)) {
                Optional<AprilTagPoseFtc> tpose = robot.vision.getBoardPose(targetTag);
                if (tpose.isPresent()) {
                    double margin = 0.0;

                    Vector2d newPose = new Vector2d(
                            robot.drive.pose.position.x +
                                    (tpose.get().y - (BoardAlignmentConstants.DistFromBoard + 1)),
                            robot.drive.pose.position.y - (tpose.get().x + BoardAlignmentConstants.XOffset)
                    );
                    Rotation2d heading = robot.drive.pose.heading.plus(
                            Math.toRadians(tpose.get().yaw)
                    );

                    aligned = true;

                    Actions.runBlocking(
                            robot.drive
                                    .actionBuilder(robot.drive.pose, TrajectorySpeed.SLOW)
                                    .strafeToLinearHeading(
                                            newPose,
                                            heading
                                    )
                                    .build()
                    );
                }
            }
        }
        return aligned;
    }
}


