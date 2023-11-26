package org.firstinspires.ftc.teamcode.Util.Classes.Vision;

import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Util.Classes.Vision.Processors.SpikeLocationDetectionProcessor;
import org.firstinspires.ftc.teamcode.Util.Enums.SpikeLocation;
import org.firstinspires.ftc.teamcode.Util.Enums.VisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class Vision {
    private final CameraName frontCamera;
    private final CameraName backCamera;

    public VisionPortal visionPortal = null;
    SpikeLocationDetectionProcessor spikeMarkDetectionProcessor = null;
    AprilTagProcessor aprilTagProcessor = null;

    public Vision(HardwareMap hardwareMap) {
        frontCamera = hardwareMap.get(WebcamName.class, "Webcam 1");
        backCamera = hardwareMap.get(WebcamName.class, "Webcam 2");
    }

    public void startProcessor(VisionProcessor processingMode) {
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCameraResolution(new Size(1280, 720));
        switch (processingMode) {
            case SPIKE_LOCATION_DETECTION:
                builder.setCamera(frontCamera);
                spikeMarkDetectionProcessor = new SpikeLocationDetectionProcessor();
                builder.addProcessor(spikeMarkDetectionProcessor);
                break;
            case APRIL_TAG_DETECTION:
                builder.setCamera(backCamera);
                aprilTagProcessor = new AprilTagProcessor.Builder().build();
                builder.addProcessor(aprilTagProcessor);
                break;
        }
        visionPortal = builder.build();
    }

    public void setProcessorEnabled(VisionProcessor processor, boolean enabled) {
        switch (processor) {
            case SPIKE_LOCATION_DETECTION:
                visionPortal.setProcessorEnabled(spikeMarkDetectionProcessor, enabled);
                break;
            case APRIL_TAG_DETECTION:
                visionPortal.setProcessorEnabled(aprilTagProcessor, enabled);
                break;
        }
    }

    public void stopProcessors() {
        visionPortal.setProcessorEnabled(spikeMarkDetectionProcessor, false);
    }

    public void toggleProcessorStreamingMode() {
        if (spikeMarkDetectionProcessor != null) spikeMarkDetectionProcessor.toggleStreamingMode();
    }

    public void update() {

    }

    public SpikeLocation getSpikeLocation() {
        if (spikeMarkDetectionProcessor != null) return spikeMarkDetectionProcessor.location;
        return SpikeLocation.CENTER;
    }

    public AprilTagProcessor getAprilTagProcessor() {
        return aprilTagProcessor;
    }

    public void close() {
        visionPortal.close();
    }
}
