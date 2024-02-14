package org.firstinspires.ftc.teamcode.util.auto.constants;

import com.acmerobotics.dashboard.config.Config;
import org.opencv.core.Scalar;

@Config
public class SpikeLocationDetectionConstants {
    public static Scalar lowerRed = new Scalar(50, 0, 0);
    public static Scalar upperRed = new Scalar(255, 50, 100);

    public static Scalar lowerBlue = new Scalar(0, 0, 80);
    public static Scalar upperBlue = new Scalar(100, 100, 255);
}
