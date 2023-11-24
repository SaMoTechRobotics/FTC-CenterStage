package org.firstinspires.ftc.teamcode.Auto.Base;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Util.Classes.AutoRobot;
import org.firstinspires.ftc.teamcode.Util.Classes.Storage.RobotStorage;
import org.firstinspires.ftc.teamcode.Util.Enums.AutoColor;
import org.firstinspires.ftc.teamcode.Util.Enums.AutoSide;
import org.firstinspires.ftc.teamcode.Util.Enums.SpikeLocation;
import org.firstinspires.ftc.teamcode.Util.Enums.VisionProcessor;

@Config
public abstract class BaseAuto extends LinearOpMode {
    private AutoRobot robot;
    private MecanumDrive drive;

    private final static Boolean Debug = true;

    private static final AutoSide SIDE = AutoSide.RIGHT;
    private static final AutoColor COLOR = AutoColor.RED;

    protected static class DriveDistances {
        public static Double ToParking = 50.0;
    }

    SpikeLocation spikeLocation = SpikeLocation.CENTER;

    @Override
    public void runOpMode() {
        RobotStorage.reset(SIDE, COLOR);
        robot = new AutoRobot(hardwareMap, telemetry);
        drive = new MecanumDrive(hardwareMap, RobotStorage.pose);

        robot.vision.startProcessor(VisionProcessor.SPIKE_LOCATION_DETECTION);
        if (Debug) {
            robot.vision.visionPortal.resumeStreaming();
        }

        ElapsedTime timer = new ElapsedTime();

        while (!isStarted()) {
            telemetry.addLine("Color: " + COLOR + " Side: " + SIDE);
            telemetry.addData("Status", "Initialized for " + Math.round(timer.seconds()));
            spikeLocation = robot.vision.getSpikeLocation();
            telemetry.addData("Spike Location", spikeLocation.toString());
            telemetry.update();
        }

        waitForStart();

        robot.vision.stopProcessors();
    }
}

