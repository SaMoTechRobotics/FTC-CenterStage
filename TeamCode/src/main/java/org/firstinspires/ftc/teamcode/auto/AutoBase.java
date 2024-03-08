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

/*
 * The base class for all autonomous programs
 * This is extended by all autonomous programs with only color and side as needed parameters
 * The field coordinate system is as follows:
 *                          RED
 *             ------------------------------
 *             |    |       | |              |
 *             |[[| |       | |             o|
 *             |[[| |       -y              o|
 *             |   /        |               o|
 *  Backstage  | /     +x   |     -x         | Audience
 *             | \          |                |
 *             |   \        |_              o|
 *             |[[| |       +y              o|
 *             |[[| |       | |             o|
 *             |    |       | |              |
 *             ------------------------------
 *                         BLUE
 */
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

    public static class StrategyConstants {
        public static DeliveryType farCycleType = DeliveryType.BACKDROP;

        public static DeliveryType cycleType = DeliveryType.BACKSTAGE;

        public enum DeliveryType {
            BACKSTAGE,
            BACKDROP
        }
    }

    public static StrategyConstants STRATEGY = new StrategyConstants();

    public static class TimingConstants {
        public static double Delay = 2;

        public static double WhiteAlignWithBoardTime = 2;
        public static double FarAlignWithBoardTime = 2.5;

        public static double CloseAlignForYellowTime = 1;

        public static double PushBoardParked = 2;

        public static double DriveIntoWhiteStack = 1.2;

        public static double DriveIntoWhiteStackCycle = 1;

        public static double StrafeAlignWithWall = 1;

        public static double CycleAlignTime = 1.5;
    }

    public static TimingConstants TIMING = new TimingConstants();

    public static class FarLocationConstants {
        public static double FarLaneY = 14;

        public static double blueStackY = 37;
        public static double redStackY = 37;

        /**
         * The locations of where the robot should be to deliver the spike mark
         * Order: Inner X, Center Y, Outer X
         */
        public static double[] blueSpikeMarks = new double[]{-29, 31, -42.5};
        public static double[] redSpikeMarks = new double[]{-30.5, 31, -44};

        // Order: Inner Y, Center Y, Outer Y
        public static double[] blueBoardY = new double[]{44.5, 38, 31};
        public static double[] redBoardY = new double[]{46, 38, 31};

        public static double boardX = 30;
        public static double moveBoardX = 32;
    }

    public static FarLocationConstants FAR_LOCATION = new FarLocationConstants();

    public static class NearLocationConstants {
        public static double CloseLaneY = 60;
        public static double BackCloseLaneY = 58.5;

        public static double WallY = 66;

        public static double stackX = -50;
        public static double blueStackY = 36.5;
        public static double redStackY = 36;

        /**
         * The locations of where the robot should be to deliver the spike mark
         * Order: Inner X, Center Y, Outer X
         */
        public static double[] blueSpikeMarks = new double[]{29.5, 35.2, 7.5};
        public static double[] redSpikeMarks = new double[]{28.5, 32, 7.5};

        /**
         * The Y coordinate of where the robot should be to deliver the spike mark
         * Order: Inner Y, Center Y, Outer Y
         */
        public static double[] blueBoardY = new double[]{42, 39, 30};
        public static double[] redBoardY = new double[]{41, 39, 30};

        public static double boardX = 34;
        public static double pushBoardX = 37;

        public static double boardCycleX = 42;

        public static double pushBoardCycleDist = 2;

        public static double pushBoardDist = 2;

        public static double ParkX = 56;
    }

    public static NearLocationConstants NEAR_LOCATION = new NearLocationConstants();

    public static class AlignmentConstants {
        // Order: Inner, Center, Outer
        public static double[] BlueOffsets = new double[]{6, 6.0, -4.8};
        public static double[] RedOffsets = new double[]{-5.2, -4.5, 6};
    }

    public static AlignmentConstants ALIGNMENT = new AlignmentConstants();

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
        RobotStorage.armUp = false;
        robot = new AutoRobot(hardwareMap, startPose);

        robot.vision.startProcessors();
        robot.vision.setColor(COLOR);

        ElapsedTime elapsedTime = new ElapsedTime();
        Timer t = new Timer();

        c = COLOR.value;
        startHeading = COLOR == AutoColor.BLUE ? 270 : 90;
        isBlue = COLOR == AutoColor.BLUE;

        ElapsedTime cameraTimer = new ElapsedTime();
        boolean aprilTagCameraWorking = false;

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Color: " + COLOR + " Side: " + SIDE);
            telemetry.addData("Status", "Initialized for " + Math.round(elapsedTime.seconds()));
            telemetry.addLine("---");

            telemetry.addData("APRIL TAG CAMERA STATE", robot.vision.aprilTagPortal.getCameraState());

            if (robot.vision.isBackCameraWorking() && !aprilTagCameraWorking) {
                robot.vision.setActiveProcessor(VisionProcessor.SPIKE_LOCATION_DETECTION);
                aprilTagCameraWorking = true;
            }

            if (cameraTimer.seconds() > 2) {
                telemetry.addData("APRIL TAG CAMERA WORKING", aprilTagCameraWorking ? "YES IT IS" : "NO, SHOW IT AN APRIL TAG");
                telemetry.addLine("---");

                BoardPosition spikeLocation = robot.vision.getSpikeLocation();
                if (spikeLocation != null) {
                    boardPosition = spikeLocation;
                    telemetry.addData("Spike Location", boardPosition.getPlace(COLOR));
                    telemetry.addData("Spike Location (RAW)", boardPosition.toString());
                } else {
                    telemetry.addData("Spike Location", "LOADING...");
                }
            } else {
                telemetry.addLine("Cameras loading...");
            }
            telemetry.update();

            sleep(10);
        }

        waitForStart();

        if (isStopRequested()) return;

        robot.drive.imu.resetYaw();

        elapsedTime.reset();

        robot.vision.setActiveProcessor(VisionProcessor.APRIL_TAG_DETECTION);

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

        RobotStorage.armUp = true;

        if (SIDE == AutoSide.FAR) {
            deliverSpikeMarkFar();

            pickUpFromWhiteStack();

            // Switch to April Tag detection
//            robot.vision.startProcessor(VisionProcessor.APRIL_TAG_DETECTION);
//            robot.vision.setActiveCamera(VisionProcessor.APRIL_TAG_DETECTION);

            deliverToBoardFromFar();
        } else {
            robot.vision.close();

            deliverBothNear();

            cycleWhiteStack(StrategyConstants.cycleType);

            robot.arm.setRotation(ArmRotation.Down);
            robot.arm.setWristRotation(WristRotation.Down);

            Actions.runBlocking(
                    robot.drive.actionBuilder(robot.drive.pose)
                            .strafeTo(new Vector2d(NearLocationConstants.ParkX, NearLocationConstants.CloseLaneY * c))
                            .build()
            );
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

        RobotStorage.armUp = false;

        while (opModeIsActive() && !isStopRequested()) {
            idle();
        }
    }

    /**
     * Delivers the purple pixel to the spike mark and then drives to be ready for stack pickup
     */
    private void deliverSpikeMarkFar() {
        double stackY = isBlue ? FarLocationConstants.blueStackY : FarLocationConstants.redStackY;

        switch (boardPosition) {
            case INNER:
                double innerX = isBlue ? FarLocationConstants.blueSpikeMarks[0] : FarLocationConstants.redSpikeMarks[0];
                Actions.runBlocking(
                        new SequentialAction(
                                robot.drive.actionBuilder(robot.drive.pose)
                                        .strafeToLinearHeading(
                                                new Vector2d(-40, 52 * c),
                                                Math.toRadians(startHeading)
                                        )
                                        .splineToSplineHeading(
                                                new Pose2d(innerX, (38 + 2) * c, Math.toRadians(startHeading + 45 * c)),
                                                Math.toRadians(startHeading + 45 * c)
                                        )
                                        .strafeToLinearHeading(
                                                new Vector2d(innerX, 38 * c),
                                                Math.toRadians(startHeading + 45 * c),
                                                robot.drive.getSpeedConstraint(TrajectorySpeed.SLOW).velConstraint
                                        ).build(),
                                new SleepAction(0.2),
                                robot.openNextClaw(),
                                new SleepAction(0.2),
                                robot.drive.actionBuilder(new Pose2d(innerX, 38 * c, Math.toRadians(startHeading + 45 * c)))
                                        .strafeToLinearHeading(
                                                new Vector2d(-36, 48 * c),
                                                Math.toRadians(startHeading + 45 * c)
                                        )
                                        .build(),
                                robot.raiseArmForStack(),
                                robot.drive.actionBuilder(new Pose2d(-36, 48 * c, Math.toRadians(startHeading + 45 * c)))
                                        .splineToLinearHeading(
                                                new Pose2d(-50, stackY * c, Math.toRadians(180)),
                                                Math.toRadians(0)
                                        )
                                        .build()
                        )
                );
                break;
            case CENTER:
                double centerY = isBlue ? FarLocationConstants.blueSpikeMarks[1] : FarLocationConstants.redSpikeMarks[1];
                Actions.runBlocking(
                        new SequentialAction(
                                robot.drive.actionBuilder(robot.drive.pose)
                                        .strafeToLinearHeading(
                                                new Vector2d(-36, 58 * c),
                                                Math.toRadians(startHeading)
                                        )
                                        .splineToSplineHeading(
                                                new Pose2d(-36, (centerY + 4) * c, Math.toRadians(startHeading)),
                                                Math.toRadians(startHeading)
                                        )
                                        .strafeToLinearHeading(
                                                new Vector2d(-36, centerY * c),
                                                Math.toRadians(startHeading),
                                                robot.drive.getSpeedConstraint(TrajectorySpeed.SLOW).velConstraint,
                                                robot.drive.getSpeedConstraint(TrajectorySpeed.SLOW).accelConstraint
                                        ).build(),
                                new SleepAction(0.2),
                                robot.openNextClaw(),
                                new SleepAction(0.2),
                                robot.drive.actionBuilder(new Pose2d(-36, centerY * c, Math.toRadians(startHeading)))
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
                double outerX = isBlue ? FarLocationConstants.blueSpikeMarks[2] : FarLocationConstants.redSpikeMarks[2];
                Actions.runBlocking(
                        new SequentialAction(
                                robot.drive.actionBuilder(robot.drive.pose)
                                        .strafeToLinearHeading(
                                                new Vector2d(-38, 54 * c),
                                                Math.toRadians(startHeading)
                                        )
                                        .splineToSplineHeading(
                                                new Pose2d(outerX, (32 + 2) * c, Math.toRadians(startHeading - 45 * c)),
                                                Math.toRadians(startHeading - 45 * c)
                                        )
                                        .strafeToLinearHeading(
                                                new Vector2d(outerX, 32 * c),
                                                Math.toRadians(startHeading - 45 * c),
                                                robot.drive.getSpeedConstraint(TrajectorySpeed.SLOW).velConstraint
                                        ).build(),
                                new SleepAction(0.2),
                                robot.openNextClaw(),
                                new SleepAction(0.2),
                                robot.drive.actionBuilder(new Pose2d(outerX, 32 * c, Math.toRadians(startHeading - 45 * c)))
                                        .strafeToLinearHeading(
                                                new Vector2d(-36, 48 * c),
                                                Math.toRadians(startHeading - 45 * c)
                                        )
                                        .build(),
                                robot.raiseArmForStack(),
                                robot.drive.actionBuilder(new Pose2d(-36, 48 * c, Math.toRadians(startHeading - 45 * c)))
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

    /**
     * Picks up the white pixel from the stack, will drive into the wall to realign for accurate pickup
     */
    private void pickUpFromWhiteStack() {
        robot.arm.setRotation(ArmRotation.StackAuto);
        robot.arm.setWristRotation(WristRotation.StackDown);

        double stackY = isBlue ? FarLocationConstants.blueStackY : FarLocationConstants.redStackY;

        robot.drive.correctHeadingWithIMU();

        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose, TrajectorySpeed.SLOW)
                        .strafeToLinearHeading(new Vector2d(-52, stackY * c), Math.toRadians(180))
                        .build()
        );

        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose, TrajectorySpeed.SLOW)
                        .strafeToLinearHeading(new Vector2d(-62, stackY * c), Math.toRadians(180))
                        .build()
        );

        robot.drive.setDrivePowersForSeconds(new PoseVelocity2d(
                new Vector2d(0.3, 0), 0
        ), TimingConstants.DriveIntoWhiteStack);

//        robot.drive.pose = new Pose2d(-65, robot.drive.pose.position.y, Math.toRadians(180));
        robot.drive.pose = new Pose2d(-65, robot.drive.pose.position.y, Math.toRadians(180));

        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose, TrajectorySpeed.SLOW)
                        .strafeToLinearHeading(new Vector2d(-63.5, robot.drive.pose.position.y), Math.toRadians(180))
                        .build()
        );

        robot.arm.setRotation(ArmRotation.StackDownAuto);
        robot.arm.setWristRotation(WristRotation.AutoPickupStack);

        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose, TrajectorySpeed.SLOW)
                        .strafeToLinearHeading(new Vector2d(-65, robot.drive.pose.position.y), Math.toRadians(180))
                        .build()
        );

        Actions.runBlocking(new SleepAction(0.2));

        robot.claw.close();

        Actions.runBlocking(new SleepAction(0.5));

        robot.arm.setRotation(ArmRotation.HoldDown);
        robot.arm.setWristRotation(WristRotation.Down);
    }

    /**
     * Drives across the field after picking up from stack and then delivers white and yellow pixels to the board using apriltag detection
     */
    private void deliverToBoardFromFar() {
        double strafeToFarLaneX = -50;
        if (boardPosition == BoardPosition.INNER) {
            strafeToFarLaneX = -42;
        } else if (boardPosition == BoardPosition.OUTER) {
            strafeToFarLaneX = -36;
        }

        // Plan target tags for delivery

        BoardPosition targetTagForSwoop = BoardPosition.CENTER;
        if (boardPosition == BoardPosition.CENTER) {
            targetTagForSwoop = BoardPosition.OUTER;
        }

        double[] alignmentOffsets = isBlue ? AlignmentConstants.BlueOffsets : AlignmentConstants.RedOffsets;

        double[] boardYLocations = isBlue ? FarLocationConstants.blueBoardY : FarLocationConstants.redBoardY;

        double swoopOffset = alignmentOffsets[boardPosition.getIndex()];

        BoardPosition whiteTag = BoardPosition.CENTER;
        switch (boardPosition) {
            case INNER:
                whiteTag = BoardPosition.OUTER;
                break;
            case CENTER:
                whiteTag = BoardPosition.INNER;
                break;
            case OUTER:
                whiteTag = BoardPosition.INNER;
                break;
        }

        double boardY = boardYLocations[whiteTag.getIndex()];

        Actions.runBlocking(
                new SequentialAction(
                        robot.drive.actionBuilder(robot.drive.pose, TrajectorySpeed.FAST)
                                .strafeToLinearHeading(new Vector2d(strafeToFarLaneX, 36 * c), Math.toRadians(180))
                                .build(),
                        robot.drive.actionBuilder(new Pose2d(strafeToFarLaneX, 36 * c, Math.toRadians(180)), TrajectorySpeed.SLOW)
                                .strafeToLinearHeading(new Vector2d(strafeToFarLaneX, FarLocationConstants.FarLaneY * c), Math.toRadians(180))
                                .build()
                )
        );

//        if (TimingConstants.Delay > 0) Actions.runBlocking(new SleepAction(TimingConstants.Delay));

//        double newHeading = robot.drive.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) + startHeading;
//        robot.drive.pose = new Pose2d(robot.drive.pose.position.x, robot.drive.pose.position.y, Math.toRadians(newHeading));
        robot.drive.correctHeadingWithIMU();


//        robot.arm.setRotation(ArmRotation.MidDeliver);
//        robot.arm.update();

        // Delivery to board

        if (StrategyConstants.farCycleType == StrategyConstants.DeliveryType.BACKSTAGE) {
            Actions.runBlocking(
                    new SequentialAction(
                            robot.drive.actionBuilder(robot.drive.pose, TrajectorySpeed.FAST)
                                    .turnTo(Math.toRadians(180))
                                    .strafeToLinearHeading(new Vector2d(FarLocationConstants.boardX, FarLocationConstants.FarLaneY * c), Math.toRadians(180))
                                    .build(),
                            robot.prepareForBackstageDelivery(),
                            robot.drive.actionBuilder(new Pose2d(FarLocationConstants.boardX, FarLocationConstants.FarLaneY * c, Math.toRadians(180)))
                                    .strafeToLinearHeading(new Vector2d(38, FarLocationConstants.FarLaneY * c), Math.toRadians(180), robot.drive.getSpeedConstraint(TrajectorySpeed.SLOW).velConstraint)
                                    .build()
                    )
            );

            robot.claw.openNext();

            Actions.runBlocking(new SleepAction(0.5 + TimingConstants.Delay));

            robot.arm.setRotation(ArmRotation.MidDeliver);
            robot.arm.setGlobalWristRotation(true);
            robot.arm.update();

            Actions.runBlocking(
                    robot.drive.actionBuilder(robot.drive.pose)
                            .strafeToLinearHeading(new Vector2d(FarLocationConstants.boardX, FarLocationConstants.FarLaneY * c), Math.toRadians(180))
                            .build()
            );

            robot.arm.setRotation(ArmRotation.PrepAutoDeliver);
            robot.arm.setGlobalWristRotation(true);
            robot.arm.update();

            Actions.runBlocking(
                    robot.drive.actionBuilder(robot.drive.pose)
                            .strafeToLinearHeading(new Vector2d(FarLocationConstants.moveBoardX, boardYLocations[targetTagForSwoop.getIndex()] * c), Math.toRadians(180))
                            .build()
            );
        } else {
            Actions.runBlocking(
                    robot.drive.actionBuilder(robot.drive.pose, TrajectorySpeed.FAST)
                            .turnTo(Math.toRadians(180))
                            .strafeToLinearHeading(new Vector2d(FarLocationConstants.boardX, FarLocationConstants.FarLaneY * c), Math.toRadians(180))
//                            .splineToConstantHeading(new Vector2d(FarLocationConstants.moveBoardX, 20 * c), Math.toRadians(180))
                            .strafeToLinearHeading(new Vector2d(FarLocationConstants.moveBoardX + 2, boardY * c), Math.toRadians(180))
                            .strafeToLinearHeading(new Vector2d(FarLocationConstants.moveBoardX, boardY * c), Math.toRadians(180), robot.drive.getSpeedConstraint(TrajectorySpeed.SLOW).velConstraint, robot.drive.getSpeedConstraint(TrajectorySpeed.SLOW).accelConstraint)
                            .build()
            );

            robot.arm.setRotation(ArmRotation.MidDeliver);
            robot.arm.setGlobalWristRotation(true);
            robot.arm.update();

            boolean whiteAligned = alignWithAprilTag(whiteTag, TimingConstants.WhiteAlignWithBoardTime, true);

            robot.arm.setRotation(ArmRotation.AutoDeliver, ArmSpeed.Mid);
            robot.arm.update();

            Rotation2d goodHeading = robot.drive.pose.heading;

            double placeOffset = 0;

            if (whiteTag == BoardPosition.INNER) {
                placeOffset += (isBlue ? 2 : 1) * c;
            } else if (whiteTag == BoardPosition.OUTER) {
                placeOffset -= 1 * c;
            }


            Actions.runBlocking(
                    robot.drive
                            .actionBuilder(robot.drive.pose, TrajectorySpeed.SLOW)
                            .strafeToLinearHeading(
                                    new Vector2d(
                                            robot.drive.pose.position.x + 2,
                                            robot.drive.pose.position.y + placeOffset
                                    ),
                                    goodHeading
                            )
                            .build()
            );

            robot.claw.openNext();

            robot.arm.setRotation(ArmRotation.PrepAutoDeliver);

            Actions.runBlocking(
                    new SequentialAction(
                            robot.drive
                                    .actionBuilder(robot.drive.pose, TrajectorySpeed.SLOW)
                                    .strafeTo(
                                            new Vector2d(
                                                    FarLocationConstants.moveBoardX,
                                                    robot.drive.pose.position.y
                                            )
                                    )
                                    .build(),
                            robot.drive
                                    .actionBuilder(robot.drive.pose, TrajectorySpeed.SLOW)
                                    .strafeToLinearHeading(
                                            new Vector2d(
                                                    FarLocationConstants.moveBoardX,
                                                    boardYLocations[targetTagForSwoop.getIndex()] * c
                                            ),
                                            goodHeading
                                    )
                                    .build()
                    )
            );
        }

        robot.arm.setRotation(ArmRotation.AutoDeliver);
        robot.arm.setBoardAngle(WristRotation.AutoBoardAngle);
        robot.arm.setGlobalWristRotation(true);
//        robot.arm.setBoardAngle(WristRotation.AutoBoardAngle);
        robot.arm.update();

        robot.arm.setRotation(ArmRotation.AutoDeliverLow, ArmSpeed.Mid);
        robot.arm.update();

        boolean aligned = alignWithAprilTag(targetTagForSwoop, TimingConstants.FarAlignWithBoardTime, true);
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

        robot.arm.setRotation(ArmRotation.AutoDeliverLow, ArmSpeed.Mid);
        robot.arm.update();

        if (!aligned) {
            robot.arm.setRotation(ArmRotation.MidDeliver);

            Actions.runBlocking(
                    robot.drive
                            .actionBuilder(robot.drive.pose, TrajectorySpeed.SLOW)
                            .strafeTo(
                                    new Vector2d(
                                            33,
                                            boardYLocations[boardPosition.getIndex()] * c
                                    )
                            )
                            .build()
            );
        } else {
            Actions.runBlocking(
                    robot.drive
                            .actionBuilder(robot.drive.pose, TrajectorySpeed.SLOW)
                            .strafeTo(
                                    new Vector2d(
                                            robot.drive.pose.position.x + 0.5,
                                            robot.drive.pose.position.y + swoopOffset
                                    )
                            )
                            .build()
            );
        }

        final double alignedY = robot.drive.pose.position.y;

        robot.arm.setRotation(ArmRotation.AutoDeliverLow, ArmSpeed.Min);
        robot.arm.update();

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
    }

    /**
     * Delivers the purple pixel to the spike mark and then delivers the yellow pixel to the board without apriltag detection
     */
    private void deliverBothNear() {
        switch (boardPosition) {
            case INNER:
                double innerX = isBlue ? NearLocationConstants.blueSpikeMarks[0] : NearLocationConstants.redSpikeMarks[0];
                Actions.runBlocking(
                        new SequentialAction(
                                robot.drive.actionBuilder(robot.drive.pose)
                                        .strafeToLinearHeading(
                                                new Vector2d(14, 54 * c),
                                                Math.toRadians(startHeading)
                                        )
                                        .strafeToLinearHeading(
                                                new Vector2d(innerX, 48 * c),
                                                Math.toRadians(180)
                                        )
                                        .strafeToLinearHeading(
                                                new Vector2d(innerX, 34 * c),
                                                Math.toRadians(180),
                                                robot.drive.getSpeedConstraint(TrajectorySpeed.SLOW).velConstraint
                                        ).build(),
                                new SleepAction(0.2),
                                robot.openNextClaw(),
                                new SleepAction(0.2),
                                robot.drive.actionBuilder(new Pose2d(innerX, 34 * c, Math.toRadians(180)))
                                        .strafeToLinearHeading(
                                                new Vector2d(innerX + 2, 34 * c),
                                                Math.toRadians(180)
                                        )
                                        .build()
                        )
                );
                break;
            case CENTER:
                double centerY = isBlue ? NearLocationConstants.blueSpikeMarks[1] : NearLocationConstants.redSpikeMarks[1];
                Actions.runBlocking(
                        new SequentialAction(
                                robot.drive.actionBuilder(robot.drive.pose)
                                        .strafeToLinearHeading(
                                                new Vector2d(12, 58 * c),
                                                Math.toRadians(startHeading)
                                        )
                                        .splineToSplineHeading(
                                                new Pose2d(12, (centerY + 2) * c, Math.toRadians(startHeading)),
                                                Math.toRadians(startHeading)
                                        )
                                        .strafeToLinearHeading(
                                                new Vector2d(12, centerY * c),
                                                Math.toRadians(startHeading),
                                                robot.drive.getSpeedConstraint(TrajectorySpeed.SLOW).velConstraint
                                        ).build(),
                                new SleepAction(0.2),
                                robot.openNextClaw(),
                                new SleepAction(0.2),
                                robot.drive.actionBuilder(new Pose2d(12, centerY * c, Math.toRadians(startHeading)))
                                        .strafeToLinearHeading(
                                                new Vector2d(12, 48 * c),
                                                Math.toRadians(startHeading)
                                        )
                                        .strafeToLinearHeading(
                                                new Vector2d(36, 36 * c),
                                                Math.toRadians(180)
                                        )
                                        .build()
                        )
                );
                break;
            case OUTER:
                double outerX = isBlue ? NearLocationConstants.blueSpikeMarks[2] : NearLocationConstants.redSpikeMarks[2];
                double angle = -60 * c;
                Actions.runBlocking(
                        new SequentialAction(
                                robot.drive.actionBuilder(robot.drive.pose)
                                        .strafeToLinearHeading(new Vector2d(14, 54 * c), Math.toRadians(startHeading))
                                        .splineToSplineHeading(
                                                new Pose2d(outerX, (35 + 2) * c, Math.toRadians(startHeading + angle)),
                                                Math.toRadians(startHeading + angle)
                                        )
                                        .strafeToLinearHeading(
                                                new Vector2d(outerX, 35 * c),
                                                Math.toRadians(startHeading + angle),
                                                robot.drive.getSpeedConstraint(TrajectorySpeed.SLOW).velConstraint
                                        ).build(),
                                new SleepAction(0.2),
                                robot.openNextClaw(),
                                new SleepAction(0.2),
                                robot.drive.actionBuilder(new Pose2d(outerX, 34 * c, Math.toRadians(startHeading + angle)))
                                        .strafeToLinearHeading(
                                                new Vector2d(10, 36 * c),
                                                Math.toRadians(180)
                                        )
                                        .build()
                        )
                );
                break;
        }

        double[] boardYLocations = isBlue ? NearLocationConstants.blueBoardY : NearLocationConstants.redBoardY;

        double boardY = boardYLocations[0];
        if (boardPosition == BoardPosition.CENTER) {
            boardY = boardYLocations[1];
        } else if (boardPosition == BoardPosition.OUTER) {
            boardY = boardYLocations[2];
        }

        robot.arm.setRotation(ArmRotation.PrepAutoDeliver);
        robot.arm.setGlobalWristRotation(true);
        robot.arm.update();

        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose)
                        .strafeToLinearHeading(new Vector2d(NearLocationConstants.boardX, boardY * c), Math.toRadians(180))
                        .build()
        );

        boolean aligned = alignWithAprilTag(boardPosition, TimingConstants.CloseAlignForYellowTime, false);


        if (aligned) {
            Actions.runBlocking(
                    robot.drive.actionBuilder(robot.drive.pose)
                            .strafeToLinearHeading(new Vector2d(robot.drive.pose.position.x + NearLocationConstants.pushBoardDist, robot.drive.pose.position.y), robot.drive.pose.heading)
                            .build()
            );
        } else {
            Actions.runBlocking(
                    robot.drive.actionBuilder(robot.drive.pose)
                            .strafeToLinearHeading(new Vector2d(NearLocationConstants.pushBoardX, boardY * c), Math.toRadians(180))
                            .build()
            );
        }

//        if (TimingConstants.CloseAlignForYellowTime > 0) {
//            alignWithAprilTag(boardPosition, TimingConstants.CloseAlignForYellowTime);
//        }

        robot.arm.setRotation(ArmRotation.CloseAutoDeliver, ArmSpeed.Min);
        robot.arm.update();

        Actions.runBlocking(new SleepAction(0.2));

        robot.claw.open();

        Actions.runBlocking(new SleepAction(0.2));
    }

    /**
     * Starts from the board and then drives across the field to pick up from the white stack and then delivers the 2 white pixels to the board or backstage based on the cycle type
     *
     * @param cycleType Where to deliver the 2 white pixels
     */
    public void cycleWhiteStack(StrategyConstants.DeliveryType cycleType) {
        robot.resetForIntake();

        Actions.runBlocking(
                new SequentialAction(
                        robot.drive.actionBuilder(robot.drive.pose)
                                .splineToConstantHeading(new Vector2d(20, NearLocationConstants.CloseLaneY * c), Math.toRadians(180))
                                .build(),
                        robot.drive.correctHeadingWithIMUAction(),
                        robot.drive.actionBuilder(new Pose2d(20, NearLocationConstants.CloseLaneY * c, Math.toRadians(180)), TrajectorySpeed.FAST)
                                .strafeToLinearHeading(new Vector2d(-36, NearLocationConstants.CloseLaneY * c), Math.toRadians(180))
                                .build()
                )
        );

        robot.drive.setDrivePowersForSeconds(new PoseVelocity2d(new Vector2d(0, -0.4 * c), 0), TimingConstants.StrafeAlignWithWall);

        robot.drive.pose = new Pose2d(robot.drive.pose.position.x, NearLocationConstants.WallY * c, Math.toRadians(180));

        double stackY = isBlue ? NearLocationConstants.blueStackY : NearLocationConstants.redStackY;

        robot.resetForIntake();

        robot.arm.setRotation(ArmRotation.Stack5, ArmSpeed.Mid);
        robot.arm.setWristRotation(WristRotation.StackDown);

        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose)
                        .strafeToLinearHeading(new Vector2d(NearLocationConstants.stackX, stackY * c), Math.toRadians(180))
                        .build()
        );

        robot.drive.setDrivePowersForSeconds(new PoseVelocity2d(
                new Vector2d(0.3, 0), 0
        ), TimingConstants.DriveIntoWhiteStackCycle);

        robot.drive.pose = new Pose2d(-65, robot.drive.pose.position.y, Math.toRadians(180));

        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose, TrajectorySpeed.SLOW)
                        .strafeToLinearHeading(new Vector2d(-64.5, robot.drive.pose.position.y), Math.toRadians(180))
                        .build()
        );

        robot.arm.setRotation(ArmRotation.Down, ArmSpeed.Min);

        Actions.runBlocking(new SleepAction(0.2));

        robot.arm.holdRotation();

        robot.claw.close();

        Actions.runBlocking(new SleepAction(0.5));

        robot.arm.setRotation(ArmRotation.HoldDown);
        robot.arm.setWristRotation(WristRotation.HoldDown);

        Actions.runBlocking(
                robot.drive.actionBuilder(robot.drive.pose, TrajectorySpeed.SLOW)
                        .strafeToLinearHeading(new Vector2d(-63, robot.drive.pose.position.y), Math.toRadians(180))
                        .build()
        );

        robot.drive.correctHeadingWithIMU();

        Actions.runBlocking(
                new SequentialAction(
                        robot.drive.actionBuilder(robot.drive.pose)
                                .strafeToLinearHeading(new Vector2d(NearLocationConstants.stackX, stackY * c), Math.toRadians(180))
                                .splineToConstantHeading(new Vector2d(-36, NearLocationConstants.BackCloseLaneY * c), Math.toRadians(180))
                                .build(),
                        robot.drive.correctHeadingWithIMUAction(),
                        robot.drive.actionBuilder(new Pose2d(-36, NearLocationConstants.BackCloseLaneY * c, Math.toRadians(180)), TrajectorySpeed.SLOW)
                                .strafeToLinearHeading(new Vector2d(-35, NearLocationConstants.BackCloseLaneY * c), Math.toRadians(180))
                                .build()
                )
        );

        switch (cycleType) {
            case BACKSTAGE:
                Actions.runBlocking(
                        new SequentialAction(
                                robot.drive.actionBuilder(robot.drive.pose, TrajectorySpeed.FAST)
                                        .strafeToLinearHeading(new Vector2d(20, NearLocationConstants.BackCloseLaneY * c), Math.toRadians(180))
                                        .strafeToLinearHeading(new Vector2d(59, 53 * c), Math.toRadians(0))
                                        .build()
                        )
                );
                Actions.runBlocking(new SleepAction(0.2));
                robot.claw.open();
                Actions.runBlocking(
                        robot.drive.actionBuilder(robot.drive.pose)
                                .strafeToLinearHeading(new Vector2d(NearLocationConstants.ParkX, 56 * c), Math.toRadians(0))
                                .build()
                );
                break;
            case BACKDROP:
                Actions.runBlocking(
                        new SequentialAction(
                                robot.drive.actionBuilder(robot.drive.pose, TrajectorySpeed.FAST)
                                        .strafeToLinearHeading(new Vector2d(20, NearLocationConstants.BackCloseLaneY * c), Math.toRadians(180))
                                        .build()
                        )
                );

                double boardY = isBlue ? NearLocationConstants.blueBoardY[1] : NearLocationConstants.redBoardY[1];

                Actions.runBlocking(
                        robot.drive.actionBuilder(robot.drive.pose)
                                .strafeToLinearHeading(new Vector2d(NearLocationConstants.boardCycleX, boardY * c), Math.toRadians(180))
                                .build()
                );

                robot.arm.setRotation(ArmRotation.HighDeliver);
                robot.arm.setGlobalWristRotation(true);
                robot.arm.update();

                alignWithAprilTag(BoardPosition.CENTER, TimingConstants.CycleAlignTime, false);

                robot.arm.setRotation(ArmRotation.MidDeliver, ArmSpeed.Min);
                robot.arm.update();

                double goodY = robot.drive.pose.position.y;

                Actions.runBlocking(
                        robot.drive.actionBuilder(robot.drive.pose)
                                .strafeToLinearHeading(new Vector2d(robot.drive.pose.position.x + NearLocationConstants.pushBoardCycleDist, robot.drive.pose.position.y), robot.drive.pose.heading)
                                .build()
                );

                robot.claw.openNext();

                robot.arm.setRotation(robot.arm.getRotation() - ArmRotation.UpAfterDrop);
                robot.arm.update();

                Actions.runBlocking(
                        robot.drive.actionBuilder(robot.drive.pose)
                                .strafeToLinearHeading(new Vector2d(robot.drive.pose.position.x + NearLocationConstants.pushBoardCycleDist, robot.drive.pose.position.y), robot.drive.pose.heading)
                                .build()
                );

                robot.claw.openNext();

                robot.arm.setRotation(robot.arm.getRotation() - ArmRotation.UpAfterDrop);
                robot.arm.update();

                Actions.runBlocking(new SleepAction(0.2));

                robot.resetForIntake();

                Actions.runBlocking(
                        robot.drive.actionBuilder(robot.drive.pose)
                                .strafeTo(new Vector2d(robot.drive.pose.position.x - 2, NearLocationConstants.CloseLaneY * c))
                                .build()
                );
                break;
        }
    }

    /**
     * Aligns the robot with the board using the April Tag detection
     *
     * @param targetTag The tag to align with
     * @param timeout   The maximum time to align
     * @return Whether the robot was aligned with the board, basically always true unless the camera is not working
     */
    public boolean alignWithAprilTag(BoardPosition targetTag, double timeout, boolean tryFinding) {
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
            } else if (tryFinding) {
                int d = Math.abs(robot.drive.pose.position.y) > 40 ? -1 : 1;
                robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, -BoardAlignmentConstants.FindSpeed * c * d), 0));
            }
        }
        return aligned;
    }

    /**
     * Pushes the board while parked to make pixels settle in the right place
     */
    private void pushBoardWhileParked() {
        robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(-0.3, 0), 0));

        ElapsedTime driveIntoBoardTimer = new ElapsedTime();

        while (driveIntoBoardTimer.seconds() < TimingConstants.PushBoardParked && !isStopRequested()) {
            idle();
            if (robot.arm.isDown() && RobotStorage.armUp) RobotStorage.armUp = false;
        }

        robot.drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }

    private void saveToLog(String message) {
        telemetry.log().add(message);
    }
}


