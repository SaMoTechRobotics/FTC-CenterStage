package org.firstinspires.ftc.teamcode.util.vision.processors;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.util.auto.AutoColor;
import org.firstinspires.ftc.teamcode.util.auto.AutoSide;
import org.firstinspires.ftc.teamcode.util.auto.BoardPosition;
import org.firstinspires.ftc.teamcode.util.auto.constants.SpikeLocationDetectionConstants;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;

public class SpikeLocationDetectionProcessor implements VisionProcessor {
    public BoardPosition location = BoardPosition.CENTER;

    private static AutoColor color = AutoColor.BLUE;
    private static AutoSide side = AutoSide.FAR;

    public Boolean streamingOverlayMode = true;

    private Mat rawMat = new Mat();
    private final Mat colMat = new Mat();
    private final Mat valMat = new Mat();

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    public void toggleStreamingMode() {
        streamingOverlayMode = !streamingOverlayMode;
    }

    public void setColor(AutoColor color) {
        this.color = color;
    }

    public void setSide(AutoSide side) {
        this.side = side;
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        rawMat = frame;
        if (!streamingOverlayMode) return null;

        // Convert the frame to RGB
        Imgproc.cvtColor(frame, colMat, Imgproc.COLOR_RGBA2RGB);

        // Filter out everything but the color wanted
        if (color == AutoColor.BLUE) {
            Core.inRange(colMat, SpikeLocationDetectionConstants.lowerBlue, SpikeLocationDetectionConstants.upperBlue, valMat);
        } else {
            Core.inRange(colMat, SpikeLocationDetectionConstants.lowerRed, SpikeLocationDetectionConstants.upperRed, valMat);
        }

        // The 3 sections of the frame
        Rect region1 = new Rect(new Point(0, 0), new Point(frame.cols() / 3, frame.rows()));
        Rect region2 = new Rect(new Point(frame.cols() / 3, 0), new Point(2 * frame.cols() / 3, frame.rows()));
        Rect region3 = new Rect(new Point(2 * frame.cols() / 3, 0), new Point(frame.cols(), frame.rows()));

//        Rect region1 = new Rect(new Point(0, 0), new Point(frame.cols() / 2, frame.rows()));
//        Rect region2 = new Rect(new Point(frame.cols() / 2, 0), new Point(frame.cols(), frame.rows()));

        // Compute the average pixel value of each region
        Scalar mean1 = Core.mean(valMat.submat(region1));
        Scalar mean2 = Core.mean(valMat.submat(region2));
        Scalar mean3 = Core.mean(valMat.submat(region3));

        // Find the region with the highest average pixel value
        double maxMean = Math.max(Math.max(mean1.val[0], mean2.val[0]), mean3.val[0]);
//        double maxMean = Math.max(mean1.val[0], mean2.val[0]);
        if (maxMean == mean1.val[0]) {
//            if (side == AutoSide.RIGHT)
//                location = BoardPosition.CENTER;
//            else if (side == AutoSide.LEFT)
//                location = BoardPosition.LEFT;
            location = color == AutoColor.BLUE ? BoardPosition.INNER : BoardPosition.OUTER;
        } else if (maxMean == mean2.val[0]) {
//            if (side == AutoSide.RIGHT)
//                location = BoardPosition.RIGHT;
//            else if (side == AutoSide.LEFT)
//                location = BoardPosition.CENTER;
            location = BoardPosition.CENTER;
        } else if (maxMean == mean3.val[0]) {
            location = color == AutoColor.BLUE ? BoardPosition.OUTER : BoardPosition.INNER;
        }

        if (streamingOverlayMode) {
            // Display the mask
            valMat.copyTo(frame);
        } else {
            // Display the raw frame
            rawMat.copyTo(frame);
        }

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

        if (location == BoardPosition.INNER) {
            x = color == AutoColor.BLUE ? onscreenWidth / 6 : 5 * onscreenWidth / 6;
        } else if (location == BoardPosition.OUTER) {
            x = color == AutoColor.BLUE ? 5 * onscreenWidth / 6 : onscreenWidth / 6;
        }

        canvas.drawRect(x - width / 2, onscreenHeight / 2 - height / 2, x + width / 2, onscreenHeight / 2 + height / 2, paint);

//        canvas.drawLine(onscreenWidth / 2, 0, onscreenWidth / 2, onscreenHeight, paint);

        for (int section = 1; section <= 3; section++) {
            paint.setColor(section == 1 ? Color.RED : section == 2 ? Color.GREEN : Color.BLUE);
            canvas.drawLine(section * onscreenWidth / 3, 0, section * onscreenWidth / 3, onscreenHeight, paint);
        }
    }

}