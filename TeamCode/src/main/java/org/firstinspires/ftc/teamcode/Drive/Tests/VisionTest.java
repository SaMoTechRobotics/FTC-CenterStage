package org.firstinspires.ftc.teamcode.Drive.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Util.Classes.Lib.GamepadButton;
import org.firstinspires.ftc.teamcode.Util.Classes.Lib.StatefulGamepad;
import org.firstinspires.ftc.teamcode.Util.Classes.Vision.Vision;
import org.firstinspires.ftc.teamcode.Util.Enums.SpikeLocation;
import org.firstinspires.ftc.teamcode.Util.Enums.VisionProcessor;

@Config
@TeleOp(name = "Vision Test", group = "Tests")
public class VisionTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Vision vision = new Vision(hardwareMap);

        StatefulGamepad gamepad1Buttons = new StatefulGamepad(gamepad1);

        vision.startProcessor(VisionProcessor.SPIKE_LOCATION_DETECTION);
        vision.visionPortal.resumeStreaming();

        SpikeLocation spikeLocation;

        waitForStart();

        while (opModeIsActive()) {
            spikeLocation = vision.getSpikeLocation();

            if (gamepad1Buttons.wasJustPressed(GamepadButton.A)) {
                vision.toggleProcessorStreamingMode();
            }

            telemetry.addData("Spike Location", spikeLocation);
            telemetry.update();

            sleep(20);
        }

        vision.close();
    }
}