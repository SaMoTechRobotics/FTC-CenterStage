package org.firstinspires.ftc.teamcode.Drive.Tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Util.Classes.Lib.GamepadButton;
import org.firstinspires.ftc.teamcode.Util.Classes.Lib.StatefulGamepad;
import org.firstinspires.ftc.teamcode.Util.Classes.Vision.Vision;
import org.firstinspires.ftc.teamcode.Util.Enums.BoardPosition;
import org.firstinspires.ftc.teamcode.Util.Enums.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
@TeleOp(name = "Vision Test", group = "Tests")
public class VisionTest extends LinearOpMode {
    public static VisionProcessor processor = VisionProcessor.APRIL_TAG_DETECTION;

    @Override
    public void runOpMode() {
        Vision vision = new Vision(hardwareMap);

        StatefulGamepad gamepad1Buttons = new StatefulGamepad(gamepad1);

        vision.startProcessor(processor);
        vision.visionPortal.resumeStreaming();

        BoardPosition boardPosition;

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1Buttons.wasJustPressed(GamepadButton.A)) {
                vision.toggleProcessorStreamingMode();
            }

            if (processor == VisionProcessor.APRIL_TAG_DETECTION) {
                for (AprilTagDetection detection : vision.getAprilTagProcessor().getDetections()) {
                    telemetry.addLine(String.format("\n==== (ID %d)", detection.id));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                }
            }


            if (processor == VisionProcessor.SPIKE_LOCATION_DETECTION) {
                boardPosition = vision.getSpikeLocation();
                telemetry.addData("Spike Location", boardPosition);
            }
            telemetry.update();

            sleep(20);
        }

        vision.close();
    }
}