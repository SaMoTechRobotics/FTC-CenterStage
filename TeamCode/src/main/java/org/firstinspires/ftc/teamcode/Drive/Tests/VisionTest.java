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
    public static VisionProcessor processor = VisionProcessor.SPIKE_LOCATION_DETECTION;

    @Override
    public void runOpMode() {
        Vision vision = new Vision(hardwareMap);

        StatefulGamepad gamepad1Buttons = new StatefulGamepad(gamepad1);

        vision.startProcessor(processor);
        vision.visionPortal.resumeStreaming();

        SpikeLocation spikeLocation;

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1Buttons.wasJustPressed(GamepadButton.A)) {
                vision.toggleProcessorStreamingMode();
            }

            if (processor == VisionProcessor.APRIL_TAG_DETECTION) {
            }


            if (processor == VisionProcessor.SPIKE_LOCATION_DETECTION) {
                spikeLocation = vision.getSpikeLocation();
                telemetry.addData("Spike Location", spikeLocation);
            }
            telemetry.update();

            sleep(20);
        }

        vision.close();
    }
}