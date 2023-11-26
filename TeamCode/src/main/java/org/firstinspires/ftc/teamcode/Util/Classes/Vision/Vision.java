package org.firstinspires.ftc.teamcode.Util.Classes.Vision;

import android.util.Size;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Util.Classes.Vision.Processors.SpikeLocationDetectionProcessor;
import org.firstinspires.ftc.teamcode.Util.Constants.Auto.BoardAlignmentConstants;
import org.firstinspires.ftc.teamcode.Util.Enums.BoardPosition;
import org.firstinspires.ftc.teamcode.Util.Enums.VisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

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

    public BoardPosition getSpikeLocation() {
        if (spikeMarkDetectionProcessor != null) return spikeMarkDetectionProcessor.location;
        return BoardPosition.CENTER;
    }

    public boolean isBoardDetected() {
        if (aprilTagProcessor == null) return false;
        return !aprilTagProcessor.getDetections().isEmpty();
    }

    public boolean isBoardDetected(BoardPosition location) {
        if (aprilTagProcessor == null) return false;
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            for (int tag : location.getTagNumbers()) {
                if (detection.id == tag) {
                    return true;
                }
            }
        }
        return false;
    }

    public Pose2d getBoardAlignedPose(Pose2d robotPose, BoardPosition location) {
        if (aprilTagProcessor == null) return robotPose;
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        if (currentDetections.isEmpty()) return robotPose;
        AprilTagDetection goalDetection = null;
        for (AprilTagDetection detection : currentDetections) {
            for (int tag : location.getTagNumbers()) {
                if (detection.id == tag) {
                    goalDetection = detection;
                    break;
                }
            }
        }
        if (goalDetection == null) return robotPose;
        AprilTagPoseFtc aprilTagPose = goalDetection.ftcPose;
        double x = robotPose.position.x - aprilTagPose.y + Math.cos(robotPose.heading.log()) * BoardAlignmentConstants.minDistanceFromBoard;
        double y = robotPose.position.y + aprilTagPose.x + Math.sin(robotPose.heading.log()) * BoardAlignmentConstants.minDistanceFromBoard;
        double heading = Math.toRadians(-aprilTagPose.yaw);
        return new Pose2d(x, y, heading);
    }

    public AprilTagProcessor getAprilTagProcessor() {
        return aprilTagProcessor;
    }

    public void close() {
        visionPortal.close();
    }
}
