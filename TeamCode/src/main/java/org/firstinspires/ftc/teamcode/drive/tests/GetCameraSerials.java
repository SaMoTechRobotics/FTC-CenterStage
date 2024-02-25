package org.firstinspires.ftc.teamcode.drive.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.vision.VisionProcessor;

@Config
@TeleOp(name = "Get Camera Serials", group = "Tests")
public class GetCameraSerials extends LinearOpMode {
    public static VisionProcessor processor = VisionProcessor.APRIL_TAG_DETECTION;

    @Override
    public void runOpMode() {
        WebcamName webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1"); // UC762
        WebcamName webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2"); // 6234BF60

        telemetry.addData("Webcam 1 Serial", webcam1.getSerialNumber().getString());
        telemetry.addData("Webcam 2 Serial", webcam2.getSerialNumber().getString());
        telemetry.update();

        waitForStart();
    }
}