package org.firstinspires.ftc.teamcode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.util.auto.AutoColor;
import org.firstinspires.ftc.teamcode.util.auto.AutoSide;
import org.firstinspires.ftc.teamcode.util.auto.BoardPosition;
import org.firstinspires.ftc.teamcode.util.auto.RobotStorage;
import org.firstinspires.ftc.teamcode.util.robot.Robot;
import org.firstinspires.ftc.teamcode.util.vision.VisionProcessor;

@Config
public abstract class AutoBase extends LinearOpMode {
    private final static Boolean Debug = true;

    private Robot robot;

    protected AutoSide SIDE = AutoSide.FAR;
    protected AutoColor COLOR = AutoColor.RED;

    protected abstract AutoSide getSide();

    protected abstract AutoColor getColor();

    BoardPosition boardPosition = BoardPosition.CENTER;

    @Override
    public void runOpMode() {
        SIDE = getSide();
        COLOR = getColor();

        Pose2d startPose = RobotStorage.getStartPose(SIDE, COLOR);
        RobotStorage.setPose(startPose);
        robot = new Robot(hardwareMap, telemetry);

        robot.vision.startProcessor(VisionProcessor.SPIKE_LOCATION_DETECTION);
        robot.vision.setColor(COLOR);

        ElapsedTime timer = new ElapsedTime();

        int c = COLOR.value;

        double startHeading = COLOR == AutoColor.BLUE ? 270 : 90;

        while ((!isStarted() || !robot.vision.isReady()) && !isStopRequested()) {
            telemetry.addLine("Color: " + COLOR + " Side: " + SIDE);
            telemetry.addData("Status", "Initialized for " + Math.round(timer.seconds()));
            telemetry.addLine("---");

            boardPosition = robot.vision.getSpikeLocation();
            telemetry.addData("Spike Location", boardPosition.toString());
            telemetry.update();
        }

        waitForStart();

        timer.reset();

        if (SIDE == AutoSide.FAR) {
            deliverSpikeMarkFar();
        } else {
            deliverSpikeMarkNear();
        }

        telemetry.addLine("Auto Finished in " + Math.round(timer.seconds()) + " seconds");
        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }
    }

    private void deliverSpikeMarkFar() {

    }

    private void deliverSpikeMarkNear() {

    }
}


