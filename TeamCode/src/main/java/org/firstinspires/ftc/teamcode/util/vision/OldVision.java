package org.firstinspires.ftc.teamcode.util.vision;

import android.util.Size;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.auto.AutoColor;
import org.firstinspires.ftc.teamcode.util.auto.BoardPosition;
import org.firstinspires.ftc.teamcode.util.auto.constants.BoardAlignmentConstants;
import org.firstinspires.ftc.teamcode.util.vision.processors.SpikeLocationDetectionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Optional;

public class OldVision {
    private final WebcamName frontCamera;
    private final WebcamName backCamera;

    public static int AprilTagCameraWidth = 640;
    public static int AprilTagCameraHeight = 480;

    public VisionPortal visionPortal = null;
    public VisionPortal visionPortal2 = null;
    SpikeLocationDetectionProcessor spikeMarkDetectionProcessor = null;
    AprilTagProcessor aprilTagProcessor = null;

    private boolean multiProcessor = false;

    public OldVision(HardwareMap hardwareMap) {
        frontCamera = hardwareMap.get(WebcamName.class, "Webcam 2");
        backCamera = hardwareMap.get(WebcamName.class, "Webcam 1");
    }

    public void startProcessor(VisionProcessor processingMode) {
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCameraResolution(new Size(1280, 720));
//        if (processingMode == VisionProcessor.APRIL_TAG_DETECTION) {
//            builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
//            builder.setCameraResolution(new Size(AprilTagCameraWidth, AprilTagCameraHeight));
//        }
//        builder.setAutoStopLiveView(true);
//        builder.enableLiveView(false);
        switch (processingMode) {
            case SPIKE_LOCATION_DETECTION:
                builder.setCamera(frontCamera);
                spikeMarkDetectionProcessor = new SpikeLocationDetectionProcessor();
                builder.addProcessor(spikeMarkDetectionProcessor);
                break;
            case APRIL_TAG_DETECTION:
                builder.setCamera(backCamera);
                aprilTagProcessor = new AprilTagProcessor.Builder()
                        .setLensIntrinsics(913.562, 913.562, 650.769, 346.753)
                        .setDrawCubeProjection(true)
                        .setDrawAxes(true)
                        .build();
                builder.addProcessor(aprilTagProcessor);
                break;
        }
        if (visionPortal == null) visionPortal = builder.build();
        else visionPortal2 = builder.build();
    }
//
//    public void startProcessors() {
//        multiProcessor = true;
//        VisionPortal.Builder builder1 = new VisionPortal.Builder();
//        builder1.setCameraResolution(new Size(1280, 720));
//        CameraName switchableCamera = ClassFactory.getInstance()
//                .getCameraManager().nameForSwitchableCamera(frontCamera, backCamera);
//        builder1.setCamera(switchableCamera);
//        builder1.setCamera(BuiltinCameraDirection.FRONT);
//        spikeMarkDetectionProcessor = new SpikeLocationDetectionProcessor();
//        builder.addProcessor(spikeMarkDetectionProcessor);
//        aprilTagProcessor = new AprilTagProcessor.Builder()
//                .setLensIntrinsics(913.562, 913.562, 650.769, 346.753)
//                .setDrawCubeProjection(true)
//                .setDrawAxes(true)
//                .build();
//        builder.addProcessor(aprilTagProcessor);
//        visionPortal = builder.build();
//        visionPortal.setActiveCamera(frontCamera);

//        multiProcessor = true;
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//        builder.setCameraResolution(new Size(1280, 720));
//        CameraName switchableCamera = ClassFactory.getInstance()
//                .getCameraManager().nameForSwitchableCamera(frontCamera, backCamera);
//        builder.setCamera(switchableCamera);
//        spikeMarkDetectionProcessor = new SpikeLocationDetectionProcessor();
//        builder.addProcessor(spikeMarkDetectionProcessor);
//        aprilTagProcessor = new AprilTagProcessor.Builder()
//                .setLensIntrinsics(913.562, 913.562, 650.769, 346.753)
//                .setDrawCubeProjection(true)
//                .setDrawAxes(true)
//                .build();
//        builder.addProcessor(aprilTagProcessor);
//        visionPortal = builder.build();
//        visionPortal.setActiveCamera(frontCamera);
//        VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);
//    }

    public void initMultiPortal() {
        multiProcessor = true;
        VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);
    }

    public void setActiveCamera(VisionProcessor processor) {
        if (!multiProcessor) return;
//        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) return;
        if (processor == VisionProcessor.APRIL_TAG_DETECTION) {
//            visionPortal.setProcessorEnabled(aprilTagProcessor, true);
//            visionPortal.setProcessorEnabled(spikeMarkDetectionProcessor, false);
            visionPortal.setActiveCamera(backCamera);
        } else {
//            visionPortal.setProcessorEnabled(aprilTagProcessor, false);
//            visionPortal.setProcessorEnabled(spikeMarkDetectionProcessor, true);
            visionPortal.setActiveCamera(frontCamera);
        }
    }

    public void setColor(AutoColor color) {
        if (spikeMarkDetectionProcessor != null) spikeMarkDetectionProcessor.setColor(color);
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
        return null;
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

    public Optional<AprilTagPoseFtc> getBoardPose(BoardPosition location) {
        if (aprilTagProcessor != null && !aprilTagProcessor.getDetections().isEmpty()) {
            for (AprilTagDetection detection : aprilTagProcessor.getDetections()) {
                for (int tag : location.getTagNumbers()) {
                    if (detection.id == tag) {
//                        return Optional.of(new Pose2d(detection.ftcPose.x, detection.ftcPose.y, Math.toRadians(detection.ftcPose.yaw)));
                        return Optional.of(detection.ftcPose);
                    }
                }
            }
        }
        return Optional.empty();
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
        double x = robotPose.position.x - aprilTagPose.y + BoardAlignmentConstants.minDistanceFromBoard;
        double y = robotPose.position.y + aprilTagPose.x;
        double heading = robotPose.heading.log() + Math.toRadians(aprilTagPose.yaw);
        return new Pose2d(x, y, heading);
    }

    public AprilTagProcessor getAprilTagProcessor() {
        return aprilTagProcessor;
    }

    public void nextPortal() {
        visionPortal.close();
        visionPortal = visionPortal2;
    }

    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    public boolean isReady() {
        return true;
//        return visionPortal.getCameraState() == VisionPortal.CameraState.CAMERA_DEVICE_READY;
    }
}
