package org.firstinspires.ftc.teamcode.Util.Classes.Vision;

import android.util.Size;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

public class Vision {
    VisionPortal visionPortal;

    public Vision(HardwareMap hardwareMap) {
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCameraResolution(new Size(1280, 720));
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        visionPortal = builder.build();
    }
}
