package org.firstinspires.ftc.teamcode.Util.Classes.Vision.Processors;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.Util.Enums.SpikeLocation;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

public class SpikeLocationDetectionProcessor implements VisionProcessor {
    public SpikeLocation location = SpikeLocation.CENTER;

    public Scalar lower = new Scalar(130, 0, 0);
    public Scalar upper = new Scalar(255, 130, 130);

    private final Mat colMat = new Mat();
    private final Mat redMat = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        // Convert the frame to RGB
        Imgproc.cvtColor(frame, colMat, Imgproc.COLOR_RGBA2RGB);

        // Filter out everything but red
        Core.inRange(colMat, lower, upper, redMat);

        // The 3 sections of the frame
        Rect region1 = new Rect(new Point(0, 0), new Point(frame.cols() / 3, frame.rows()));
        Rect region2 = new Rect(new Point(frame.cols() / 3, 0), new Point(2 * frame.cols() / 3, frame.rows()));
        Rect region3 = new Rect(new Point(2 * frame.cols() / 3, 0), new Point(frame.cols(), frame.rows()));

        // Compute the average pixel value of each region
        Scalar mean1 = Core.mean(redMat.submat(region1));
        Scalar mean2 = Core.mean(redMat.submat(region2));
        Scalar mean3 = Core.mean(redMat.submat(region3));

        // Find the region with the highest average pixel value
        double maxMean = Math.max(Math.max(mean1.val[0], mean2.val[0]), mean3.val[0]);
        if (maxMean == mean1.val[0]) {
            location = SpikeLocation.LEFT;
        } else if (maxMean == mean2.val[0]) {
            location = SpikeLocation.CENTER;
        } else if (maxMean == mean3.val[0]) {
            location = SpikeLocation.RIGHT;
        }

        // Display the mask
        redMat.copyTo(frame);

        return null; // No context object
    }


    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint paint = new Paint();
        paint.setColor(Color.GREEN);
        paint.setStyle(Paint.Style.FILL);
        paint.setStrokeWidth(5);

        int width = 50;
        int height = 50;

        int x = onscreenWidth / 2;

        if (location == SpikeLocation.LEFT) {
            x = onscreenWidth / 6;
        } else if (location == SpikeLocation.RIGHT) {
            x = 5 * onscreenWidth / 6;
        }

        canvas.drawRect(x - width / 2, onscreenHeight / 2 - height / 2, x + width / 2, onscreenHeight / 2 + height / 2, paint);

        for (int section = 1; section <= 3; section++) {
            paint.setColor(section == 1 ? Color.RED : section == 2 ? Color.GREEN : Color.BLUE);
            canvas.drawLine(section * onscreenWidth / 3, 0, section * onscreenWidth / 3, onscreenHeight, paint);
        }
    }

}