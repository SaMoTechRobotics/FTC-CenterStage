package org.firstinspires.ftc.teamcode.Util.Classes.Vision;

import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Util.Classes.Vision.Processors.SpikeLocationDetectionProcessor;
import org.firstinspires.ftc.teamcode.Util.Enums.SpikeLocation;
import org.firstinspires.ftc.teamcode.Util.Enums.VisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;

public class Vision {
    private final WebcamName webcam;

    public VisionPortal visionPortal = null;
    SpikeLocationDetectionProcessor spikeMarkDetectionProcessor = null;

    public Vision(HardwareMap hardwareMap) {
        webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
    }

    public void startProcessor(VisionProcessor processingMode) {
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCameraResolution(new Size(1280, 720));
        builder.setCamera(webcam);
        switch (processingMode) {
            case SPIKE_LOCATION_DETECTION:
                spikeMarkDetectionProcessor = new SpikeLocationDetectionProcessor();
                builder.addProcessor(spikeMarkDetectionProcessor);
                break;
            case APRIL_TAG_DETECTION:
                break;
        }
        visionPortal = builder.build();
    }

    public void update() {

    }

    public SpikeLocation getSpikeLocation() {
        if (spikeMarkDetectionProcessor != null) return spikeMarkDetectionProcessor.location;
        return SpikeLocation.CENTER;
    }

    public void close() {
        visionPortal.close();
    }
}
