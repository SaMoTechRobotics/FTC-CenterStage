package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.roadrunner.speed.TrajectorySpeed;
import org.firstinspires.ftc.teamcode.util.auto.AutoColor;
import org.firstinspires.ftc.teamcode.util.auto.AutoSide;
import org.firstinspires.ftc.teamcode.util.auto.BoardPosition;
import org.firstinspires.ftc.teamcode.util.auto.RobotStorage;
import org.firstinspires.ftc.teamcode.util.robot.AutoRobot;
import org.firstinspires.ftc.teamcode.util.robot.arm.ArmRotation;
import org.firstinspires.ftc.teamcode.util.vision.VisionProcessor;

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

    public static double FarLaneY = 13;

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

        telemetry.addLine("Auto Finished in " + Math.round(elapsedTime.seconds()) + " seconds");
        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }
    }

    private void deliverSpikeMarkFar() {
        double startHeading = COLOR == AutoColor.BLUE ? 270 : 90;
        int c = COLOR.value;

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
                                                robot.drive.getSpeedConstraint(TrajectorySpeed.SLOW).velConstraint,
                                                robot.drive.getSpeedConstraint(TrajectorySpeed.SLOW).accelConstraint
                                        ).build(),
                                robot.openNextClaw(),
                                robot.drive.actionBuilder(new Pose2d(leftX, 38 * c, Math.toRadians(startHeading + 45)))
                                        .strafeToLinearHeading(
                                                new Vector2d(c == 1 ? -36 : -40, 48 * c),
                                                Math.toRadians(startHeading + 45)
                                        )
                                        .turn(Math.toRadians(-45))
                                        .strafeToLinearHeading(new Vector2d(
                                                -50, FarLaneY * c), Math.toRadians(180)
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
                                                new Vector2d(-36, 13 * c),
                                                Math.toRadians(startHeading - 180)
                                        )
                                        .strafeToLinearHeading(
                                                new Vector2d(-36, 13 * c),
                                                Math.toRadians(startHeading - 180)
                                        )
                                        .build(),
                                robot.openNextClaw(),
                                robot.drive
                                        .actionBuilder(robot.drive.pose)
                                        .lineToY(9 * c)
                                        .strafeToLinearHeading(
                                                new Vector2d(-30, FarLaneY * c),
                                                Math.toRadians(180)
                                        )
                                        .build()
                        )
                );
                break;
            case RIGHT:
                double rightX = -36 - (c == 1 ? 6 : -6);
                Actions.runBlocking(
                        new SequentialAction(
                                robot.drive
                                        .actionBuilder(robot.drive.pose)
                                        .strafeToLinearHeading(
                                                new Vector2d(-36, 48 * c),
                                                Math.toRadians(startHeading)
                                        )
                                        .splineToSplineHeading(
                                                new Pose2d(rightX, 38 * c, Math.toRadians(startHeading - 45)),
                                                Math.toRadians(startHeading + 45)
                                        )
                                        .build(),
                                robot.drive
                                        .actionBuilder(robot.drive.pose, TrajectorySpeed.SLOW)
                                        .strafeToLinearHeading(
                                                new Vector2d(rightX, 38 * c),
                                                Math.toRadians(startHeading - 45)
                                        )
                                        .build(),
                                robot.openNextClaw(),
                                robot.drive.actionBuilder(robot.drive.pose)
                                        .strafeToLinearHeading(
                                                new Vector2d(c == 1 ? -36 : -40, 48 * c),
                                                Math.toRadians(startHeading - 45)
                                        )
                                        .turn(Math.toRadians(45))
                                        .build()
                        )
                );
                break;
        }
    }

    private void deliverSpikeMarkNear() {

    }
}


