package org.firstinspires.ftc.teamcode.Drive.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Util.Classes.Vision.Vision;
import org.firstinspires.ftc.teamcode.Util.Enums.SpikeLocation;
import org.firstinspires.ftc.teamcode.Util.Enums.VisionProcessor;

@Config
@TeleOp(name = "Vision Test", group = "Tests")
public class VisionTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Vision vision = new Vision(hardwareMap);

        GamepadEx Gamepad1 = new GamepadEx(gamepad1);

        vision.startProcessor(VisionProcessor.SPIKE_LOCATION_DETECTION);
        vision.visionPortal.resumeStreaming();

        SpikeLocation spikeLocation;

        waitForStart();

        while (opModeIsActive()) {
            spikeLocation = vision.getSpikeLocation();

            if (Gamepad1.wasJustPressed(GamepadKeys.Button.A)) {
                vision.toggleProcessorStreamingMode();
            }

            telemetry.addData("Spike Location", spikeLocation);
            telemetry.update();

            sleep(20);
        }

        vision.close();
    }
}