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
    private AutoRobot robot;

    protected AutoColor COLOR;
    protected AutoSide SIDE;

    public AutoBase(AutoColor color, AutoSide side) {
        COLOR = color;
        SIDE = side;
    }

    BoardPosition boardPosition = BoardPosition.CENTER;

    private static class Timings {
        public static double Delay = 0.5;

        public static double WhiteAlignWithBoardTime = 1.5;
        public static double AlignWithBoardTime = 2.5;

        public static double PushBoardParked = 2;
    }

    public static Timings TRAJECTORY_SPEEDS = new Timings();

    public static class Locations {
        public static double FarLaneY = 14;

        public static double blueStackY = 38.6;
        public static double redStackY = 38;
    }

//    public static Locations LOCATIONS = new Locations();

    public static double Delay = 0.5;

    public static double FarLaneY = 14;

    // Right, Center, Left on blue side
    public static double[] AlignmentOffsetsBLUE = new double[]{-5.5, 6.0, 6.7};
    // Left Center Right on red side
    public static double[] AlignmentOffsetsRED = new double[]{5.5, -5.0, -4.5};

    public static double WhiteAlignWithBoardTime = 1.5;
    public static double AlignWithBoardTime = 2.5;
    public static double PushBoardTime = 0.5;

    public static double ParkX = 58;


    /**
     * This is to flip the sign of the y coordinate based on the color
     * Can be used like this:
     *
     * @implNote new Vector2d(30, 36 * c)
     */
    int c;

    /**
     * This is to use for the starting heading, facing away from the wall but flipped based on the color
     *
     * @implNote Math.toRadians(startHeading)
     */
    double startHeading;

    /**
     * Pretty self-explanatory, is blue?
     */
    boolean isBlue;

    @Override
    public void runOpMode() {
        Pose2d startPose = RobotStorage.getStartPose(SIDE, COLOR);
        RobotStorage.setPose(startPose);
        robot = new AutoRobot(hardwareMap, startPose);

        robot.vision.startProcessor(VisionProcessor.SPIKE_LOCATION_DETECTION);
        robot.vision.setColor(COLOR);

        ElapsedTime elapsedTime = new ElapsedTime();
        Timer t = new Timer();

        c = COLOR.value;
        startHeading = COLOR == AutoColor.BLUE ? 270 : 90;
        isBlue = COLOR == AutoColor.BLUE;

        while ((!isStarted() || !robot.vision.isReady()) && !isStopRequested()) {
            telemetry.addLine("Color: " + COLOR + " Side: " + SIDE);
            telemetry.addData("Status", "Initialized for " + Math.round(elapsedTime.seconds()));
            telemetry.addLine("---");

            BoardPosition spikeLocation = robot.vision.getSpikeLocation();
            if (spikeLocation != null) {
                boardPosition = spikeLocation;
                telemetry.addData("Spike Location", boardPosition.toString());
            } else {
                telemetry.addData("Spike Location", "LOADING...");
            }
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

            pickUpFromWhiteStack();

            // Switch to April Tag detection
            robot.vision.startProcessor(VisionProcessor.APRIL_TAG_DETECTION);
            robot.vision.setActiveCamera(VisionProcessor.APRIL_TAG_DETECTION);

            deliverToBoardFromFar();
        } else {
            deliverSpikeMarkNear();
        }

        String log = "Auto Finished in " + Math.round(elapsedTime.seconds() * 1000) / 1000 + " seconds";
        telemetry.addLine(log);
        telemetry.update();

        saveToLog(log);
        saveToLog("Color: " + COLOR + " Side: " + SIDE);
        saveToLog("Board Position: " + boardPosition.toString());
        saveToLog("End Pose: " + robot.drive.pose.toString());

        if (SIDE == AutoSide.FAR) {
            pushBoardWhileParked();
        }

        while (opModeIsActive() && !isStopRequested()) {
            idle();
        }
    }

    private void deliverSpikeMarkFar() {
        double stackY = isBlue ? Locations.blueStackY : Locations.redStackY;

        switch (boardPosition) {
            case INNER:
                double leftX = -36 + (COLOR == AutoColor.BLUE ? 7 : -7);
                double leftY = COLOR == AutoColor.BLUE ? 38 : 32;
                Actions.runBlocking(
                        new SequentialAction(
                                robot.drive.actionBuilder(robot.drive.pose)
                                        .strafeToLinearHeading(
                                                new Vector2d(-36, 54 * c),
                                                Math.toRadians(startHeading)
                                        )
                                        .splineToSplineHeading(
                                                new Pose2d(leftX, (leftY + 2) * c, Math.toRadians(startHeading + 45)),
                                                Math.toRadians(startHeading + 45)
                                        )
                                        .strafeToLinearHeading(
                                                new Vector2d(leftX, leftY * c),
                                                Math.toRadians(startHeading + 45),
                                                robot.drive.getSpeedConstraint(TrajectorySpeed.SLOW).velConstraint
                                        ).build(),
                                new SleepAction(0.2),
                                robot.openNextClaw(),
                                new SleepAction(0.2),
                                robot.drive.actionBuilder(new Pose2d(leftX, leftY * c, Math.toRadians(startHeading + 45)))
                                        .strafeToLinearHeading(
                                                new Vector2d(-36, 48 * c),
                                                Math.toRadians(startHeading + 45)
                                        )
                                        .build(),
                                robot.raiseArmForStack(),
                                robot.drive.actionBuilder(new Pose2d(-36, 48 * c, Math.toRadians(startHeading + 45)))
                                        .splineToLinearHeading(
                                                new Pose2d(-50, stackY * c, Math.toRadians(180)),
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
                                                new Vector2d(-36, 58 * c),
                                                Math.toRadians(startHeading)
                                        )
                                        .splineToSplineHeading(
                                                new Pose2d(-36, 34 * c, Math.toRadians(startHeading)),
                                                Math.toRadians(startHeading)
                                        )
                                        .strafeToLinearHeading(
                                                new Vector2d(-36, 32 * c),
                                                Math.toRadians(startHeading),
                                                robot.drive.getSpeedConstraint(TrajectorySpeed.SLOW).velConstraint
                                        ).build(),
                                new SleepAction(0.2),
                                robot.openNextClaw(),
                                new SleepAction(0.2),
                                robot.drive.actionBuilder(new Pose2d(-36, 32 * c, Math.toRadians(startHeading)))
                                        .strafeToLinearHeading(
                                                new Vector2d(-36, 48 * c),
                                                Math.toRadians(startHeading)
                                        )
                                        .build(),
                                robot.raiseArmForStack(),
                                robot.drive.actionBuilder(new Pose2d(-36, 48 * c, Math.toRadians(startHeading)))
                                        .splineToLinearHeading(
                                                new Pose2d(-50, stackY * c, Math.toRadians(180)),
                                                Math.toRadians(0)
                                        )
                                        .build()
                        )
                );
                break;
            case OUTER:
                double rightX = -36 - (COLOR == AutoColor.BLUE ? 6 : -6);
                double rightY = COLOR == AutoColor.BLUE ? 32 : 38;
                Actions.runBlocking(
                        new SequentialAction(
                                robot.drive.actionBuilder(robot.drive.pose)
                                        .strafeToLinearHeading(
                                                new Vector2d(c == 1 ? -36 : -38, 54 * c),
                                                Math.toRadians(startHeading)
                                        )
                                        .splineToSplineHeading(
                                                new Pose2d(rightX, (rightY + 2) * c, Math.toRadians(startHeading - 45)),
                                                Math.toRadians(startHeading - 45)
                                        )
                                        .strafeToLinearHeading(
                                                new Vector2d(rightX, rightY * c),
                                                Math.toRadians(startHeading - 45),
                                                robot.drive.getSpeedConstraint(TrajectorySpeed.SLOW).velConstraint
                                        ).build(),
                                new SleepAction(0.2),
                                robot.openNextClaw(),
                                new SleepAction(0.2),
                                robot.drive.actionBuilder(new Pose2d(rightX, rightY * c, Math.toRadians(startHeading - 45)))
                                        .strafeToLinearHeading(
                                                new Vector2d(-36, 48 * c),
                                                Math.toRadians(startHeading - 45)
                                        )
                                        .build(),
                                robot.raiseArmForStack(),
                                robot.drive.actionBuilder(new Pose2d(-36, 48 * c, Math.toRadians(startHeading - 45)))
                                        .splineToLinearHeading(
                                                new Pose2d(-50 + 2, stackY * c, Math.toRadians(180)),
                                                Math.toRadians(0)
                                        )
                                        .build(),
                                robot.drive.actionBuilder(new Pose2d(-50 + 2, stackY * c, Math.toRadians(180)), TrajectorySpeed.SLOW)
                                        .strafeToLinearHeading(
                                                new Vector2d(-50, stackY * c),
                                                Math.toRadians(180)
                                        )
                                        .build()
                        )
                );
                break;
        }
    }

    private void deliverSpikeMarkNear() {
        switch (boardPosition) {
            case INNER:
                double leftX = -36 + (COLOR == AutoColor.BLUE ? 7 : -7);
                double leftY = COLOR == AutoColor.BLUE ? 38 : 32;
                Actions.runBlocking(
                        new SequentialAction(
                                robot.drive.actionBuilder(robot.drive.pose)
                                        .strafeToLinearHeading(
                                                new Vector2d(-36, 54 * c),
                                                Math.toRadians(startHeading)
                                        )
                                        .splineToSplineHeading(
                                                new Pose2d(leftX, (leftY + 2) * c, Math.toRadians(startHeading + 45)),
                                                Math.toRadians(startHeading + 45)
                                        )
                                        .strafeToLinearHeading(
                                                new Vector2d(leftX, leftY * c),
                                                Math.toRadians(startHeading + 45),
                                                robot.drive.getSpeedConstraint(TrajectorySpeed.SLOW).velConstraint
                                        ).build(),
                                new SleepAction(0.2),
                                robot.openNextClaw(),
                                new SleepAction(0.2),
                                robot.drive.actionBuilder(new Pose2d(leftX, leftY * c, Math.toRadians(startHeading + 45)))
                                        .strafeToLinearHeading(
                                                new Vector2d(-36, 48 * c),
                                                Math.toRadians(startHeading + 45)
                                        )
                                        .build(),
                                robot.raiseArmForStack(),
                                robot.drive.actionBuilder(new Pose2d(-36, 48 * c, Math.toRadians(startHeading + 45)))
                                        .splineToLinearHeading(
                                                new Pose2d(-50, 36 * c, Math.toRadians(180)),
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
                                                new Vector2d(-36, 58 * c),
                                                Math.toRadians(startHeading)
                                        )
                                        .splineToSplineHeading(
                                                new Pose2d(-36, 34 * c, Math.toRadians(startHeading)),
                                                Math.toRadians(startHeading)
                                        )
                                        .strafeToLinearHeading(
                                                new Vector2d(-36, 32 * c),
                                                Math.toRadians(startHeading),
                                                robot.drive.getSpeedConstraint(TrajectorySpeed.SLOW).velConstraint
                                        ).build(),
                                new SleepAction(0.2),
                                robot.openNextClaw(),
                                new SleepAction(0.2),
                                robot.drive.actionBuilder(new Pose2d(-36, 32 * c, Math.toRadians(startHeading)))
                                        .strafeToLinearHeading(
                                                new Vector2d(-36, 48 * c),
                                                Math.toRadians(startHeading)
                                        )
                                        .build(),
                                robot.raiseArmForStack(),
                                robot.drive.actionBuilder(new Pose2d(-36, 48 * c, Math.toRadians(startHeading)))
                                        .splineToLinearHeading(
                                                new Pose2d(-50, 36 * c, Math.toRadians(180)),
                                                Math.toRadians(0)
                                        )
                                        .build()
                        )
                );
                break;
            case OUTER:
                double rightX = -36 - (COLOR == AutoColor.BLUE ? 6 : -6);
                double rightY = COLOR == AutoColor.BLUE ? 32 : 38;
                Actions.runBlocking(
                        new SequentialAction(
                                robot.drive.actionBuilder(robot.drive.pose)
                                        .strafeToLinearHeading(
                                                new Vector2d(c == 1 ? -36 : -38, 54 * c),
                                                Math.toRadians(startHeading)
                                        )
                                        .splineToSplineHeading(
                                                new Pose2d(rightX, (rightY + 2) * c, Math.toRadians(startHeading - 45)),
                                                Math.toRadians(startHeading - 45)
                                        )
                                        .strafeToLinearHeading(
                                                new Vector2d(rightX, rightY * c),
                                                Math.toRadians(startHeading - 45),
                                                robot.drive.getSpeedConstraint(TrajectorySpeed.SLOW).velConstraint
                                        ).build(),
                                new SleepAction(0.2),
                                robot.openNextClaw(),
                                new SleepAction(0.2),
                                robot.drive.actionBuilder(new Pose2d(rightX, rightY * c, Math.toRadians(startHeading - 45)))
                                        .strafeToLinearHeading(
                                                new Vector2d(-36, 48 * c),
                                                Math.toRadians(startHeading - 45)
                                        )
                                        .build(),
                                robot.raiseArmForStack(),
                                robot.drive.actionBuilder(new Pose2d(-36, 48 * c, Math.toRadians(startHeading - 45)))
                                        .splineToLinearHeading(
                                                new Pose2d(-50 + 2, 36 * c, Math.toRadians(180)),
                                                Math.toRadians(0)
                                        )
                                        .build(),
                                robot.drive.actionBuilder(new Pose2d(-50 + 2, 36 * c, Math.toRadians(180)), TrajectorySpeed.SLOW)
                                        .strafeToLinearHeading(
                                                new Vector2d(-50, 36 * c),
                                                Math.toRadians(180)
                                        )
                                        .build()
                        )
                );
                break;
        }
    }

    private void pickUpFromWhiteStack() {
        robot.arm.setRotation(ArmRotation.StackAuto);
        robot.arm.setWristRotation(WristRotation.StackDown);

        double stackY = isBlue ? Locations.blueStackY : Locations.redStackY;

        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose, TrajectorySpeed.SLOW)
                        .strafeToLinearHeading(new Vector2d(-62, stackY * c), Math.toRadians(180))
                        .build()
        );

        robot.drive.setDrivePowersForSeconds(new PoseVelocity2d(
                new Vector2d(0.35, 0), 0
        ), 0.6);

        robot.drive.pose = new Pose2d(-65, robot.drive.pose.position.y, Math.toRadians(180));

        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose, TrajectorySpeed.SLOW)
                        .strafeToLinearHeading(new Vector2d(-63.5, stackY * c), Math.toRadians(180))
                        .build()
        );

        robot.arm.setRotation(ArmRotation.StackDownAuto);
        robot.arm.setWristRotation(WristRotation.AutoPickupStack);

        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose, TrajectorySpeed.SLOW)
                        .strafeToLinearHeading(new Vector2d(-65, stackY * c), Math.toRadians(180))
                        .build()
        );

        Actions.runBlocking(new SleepAction(0.2));

        robot.claw.close();

        Actions.runBlocking(new SleepAction(0.5));

        robot.arm.setRotation(ArmRotation.HoldDown);
        robot.arm.setWristRotation(WristRotation.Down);
    }

    private void deliverToBoardFromFar() {
        // Drive across field

        double strafeToFarLaneX = -50;
        if (COLOR == AutoColor.BLUE) {
            if (boardPosition == BoardPosition.INNER) {
                strafeToFarLaneX = -42;
            } else if (boardPosition == BoardPosition.OUTER) {
                strafeToFarLaneX = -36;
            }
        } else {
            if (boardPosition == BoardPosition.INNER) {
                strafeToFarLaneX = -34;
            } else if (boardPosition == BoardPosition.OUTER) {
                strafeToFarLaneX = -42;
            }
        }

        Actions.runBlocking(
                new SequentialAction(
                        robot.drive.actionBuilder(robot.drive.pose)
                                .strafeToLinearHeading(new Vector2d(strafeToFarLaneX, 36 * c), Math.toRadians(180))
                                .strafeToLinearHeading(new Vector2d(strafeToFarLaneX, FarLaneY * c), Math.toRadians(180))
                                .build(),
                        new SleepAction(Delay),
                        robot.drive.actionBuilder(new Pose2d(strafeToFarLaneX, FarLaneY * c, Math.toRadians(180)), TrajectorySpeed.FAST)
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
                targetTag = BoardPosition.OUTER;
            } else {
                targetTag = BoardPosition.INNER;
            }
        } else {
            if (COLOR == AutoColor.BLUE) {
                if (boardPosition == BoardPosition.INNER) {
                    alignmentOffset = AlignmentOffsetsBLUE[2];
                } else {
                    alignmentOffset = AlignmentOffsetsBLUE[0];
                }
            } else {
                if (boardPosition == BoardPosition.INNER) {
                    alignmentOffset = AlignmentOffsetsRED[0];
                } else {
                    alignmentOffset = AlignmentOffsetsRED[2];
                }
            }
        }

        robot.claw.setFingerEnabled(true);

        robot.arm.setRotation(ArmRotation.PrepAutoDeliver);
        robot.arm.update();

        BoardPosition whiteTag = COLOR == AutoColor.BLUE ? BoardPosition.OUTER : BoardPosition.INNER;

        boolean weirdPos = ((COLOR == AutoColor.BLUE && boardPosition == BoardPosition.OUTER) ||
                (COLOR == AutoColor.RED && boardPosition == BoardPosition.INNER));

        if (weirdPos) {
            whiteTag = BoardPosition.CENTER;
            Actions.runBlocking(
                    robot.drive
                            .actionBuilder(robot.drive.pose, TrajectorySpeed.SLOW)
                            .strafeToLinearHeading(new Vector2d(30, 36 * c), Math.toRadians(180))
                            .build()
            );
        }


        boolean whiteAligned = alignWithAprilTag(whiteTag, WhiteAlignWithBoardTime);


        robot.arm.setRotation(ArmRotation.AutoDeliver, ArmSpeed.Mid);
        robot.arm.update();

        Rotation2d goodHeading = robot.drive.pose.heading;

        double yOffset = 0;

        if (whiteTag == BoardPosition.INNER) {
            yOffset -= 2 * c;
        } else if (whiteTag == BoardPosition.OUTER) {
            yOffset += 2 * c;
        }

        if (weirdPos) {
            yOffset += 4 * c;
        }

        Actions.runBlocking(
                robot.drive
                        .actionBuilder(robot.drive.pose, TrajectorySpeed.SLOW)
                        .strafeToLinearHeading(
                                new Vector2d(
                                        robot.drive.pose.position.x + 2,
                                        robot.drive.pose.position.y + yOffset
                                ),
                                goodHeading
                        )
                        .build()
        );

        robot.claw.openNext();

        Actions.runBlocking(
                new SequentialAction(
                        robot.drive
                                .actionBuilder(robot.drive.pose, TrajectorySpeed.SLOW)
                                .strafeTo(
                                        new Vector2d(
                                                robot.drive.pose.position.x - 2,
                                                robot.drive.pose.position.y
                                        )
                                )
                                .build(),
                        robot.drive
                                .actionBuilder(robot.drive.pose, TrajectorySpeed.SLOW)
                                .strafeToLinearHeading(
                                        new Vector2d(
                                                32,
                                                robot.drive.pose.position.y + ((weirdPos ? -6 : 4) * c) + (boardPosition == BoardPosition.CENTER ? 6 * c : 0)
                                        ),
                                        goodHeading
                                )
                                .build()
                )
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

        robot.arm.setRotation(ArmRotation.AutoDeliverLow, ArmSpeed.Min);

        Actions.runBlocking(
                robot.drive
                        .actionBuilder(robot.drive.pose, TrajectorySpeed.SLOW)
                        .strafeTo(
                                new Vector2d(
                                        robot.drive.pose.position.x + 1.5,
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
    }

    private void pushBoardWhileParked() {
        robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-0.3, 0), 0));

        ElapsedTime driveIntoBoardTimer = new ElapsedTime();

        while (driveIntoBoardTimer.seconds() < Timings.PushBoardParked && !isStopRequested()) {
            idle();
        }

        robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
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
                            robot.drive.pose.position.y - (tpose.get().x)
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
            } else {
                robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, -BoardAlignmentConstants.FindSpeed * c), 0));
            }
        }
        return aligned;
    }

    private void saveToLog(String message) {
        telemetry.log().add(message);
    }
}


