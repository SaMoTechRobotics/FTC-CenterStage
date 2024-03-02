package org.firstinspires.ftc.teamcode.drive.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.util.auto.BoardPosition;
import org.firstinspires.ftc.teamcode.util.lib.GamepadButton;
import org.firstinspires.ftc.teamcode.util.lib.StatefulGamepad;
import org.firstinspires.ftc.teamcode.util.vision.Vision;
import org.firstinspires.ftc.teamcode.util.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
@TeleOp(name = "Vision Test", group = "Tests")
public class VisionTest extends LinearOpMode {
//    public static VisionProcessor processor = VisionProcessor.APRIL_TAG_DETECTION;

//    public static boolean multiProcessor = true;

    @Override
    public void runOpMode() {
        Vision vision = new Vision(hardwareMap);

        StatefulGamepad gamepad1Buttons = new StatefulGamepad(gamepad1);

        vision.startProcessors();
//
//        if (!multiProcessor) {
//            vision.startProcessor(processor);
//            vision.visionPortal.resumeStreaming();
//        } else {
//            vision.initMultiPortal();
//            vision.startProcessor(VisionProcessor.APRIL_TAG_DETECTION);
//            vision.startProcessor(VisionProcessor.SPIKE_LOCATION_DETECTION);
//        }

        BoardPosition boardPosition;

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            gamepad1Buttons.update();

//            if (gamepad1Buttons.wasJustPressed(GamepadButton.Y)) {
//                vision.setActiveCamera(VisionProcessor.SPIKE_LOCATION_DETECTION);
//            } else if (gamepad1Buttons.wasJustPressed(GamepadButton.X)) {
//                vision.setActiveCamera(VisionProcessor.APRIL_TAG_DETECTION);
//            }
//
//            if (processor == VisionProcessor.APRIL_TAG_DETECTION || multiProcessor) {
//                for (AprilTagDetection detection : vision.getAprilTagProcessor().getDetections()) {
//                    telemetry.addLine(String.format("\n==== (ID %d)", detection.id));
//                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
//                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
//                }
//            }
//            if (processor == VisionProcessor.SPIKE_LOCATION_DETECTION || multiProcessor) {
//                boardPosition = vision.getSpikeLocation();
//                telemetry.addData("Spike Location", boardPosition);
//            }

            if (gamepad1Buttons.wasJustPressed(GamepadButton.Y)) {
                vision.setActiveProcessor(VisionProcessor.SPIKE_LOCATION_DETECTION);
            } else if (gamepad1Buttons.wasJustPressed(GamepadButton.A)) {
                vision.setActiveProcessor(VisionProcessor.APRIL_TAG_DETECTION);
            }

            for (AprilTagDetection detection : vision.getAprilTagProcessor().getDetections()) {
                if (detection != null && detection.ftcPose != null) {
                    telemetry.addLine(String.format("\n==== (ID %d)", detection.id));
                    telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                    telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                }
            }
            telemetry.addLine("---");
            boardPosition = vision.getSpikeLocation();
            telemetry.addData("Spike Location", boardPosition);
            telemetry.update();
        }

        vision.close();
    }
}