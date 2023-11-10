package org.firstinspires.ftc.teamcode.Drive.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Util.Classes.AutoRobot;
import org.firstinspires.ftc.teamcode.Util.Enums.VisionProcessor;


@Config
@TeleOp(name = "Vision Test", group = "Tests")
public class VisionTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        AutoRobot robot = new AutoRobot(hardwareMap, telemetry);

        GamepadEx Gamepad1 = new GamepadEx(gamepad1);
        GamepadEx Gamepad2 = new GamepadEx(gamepad2);

        robot.vision.startProcessor(VisionProcessor.SPIKE_LOCATION_DETECTION);


        while (!isStarted()) {
            telemetry.addData("Spike Location", robot.vision.spikeLocation);
            telemetry.update();
        }
        waitForStart();

        while (opModeIsActive()) {
            robot.vision.update();

        }
    }
}