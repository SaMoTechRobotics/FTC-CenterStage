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

public class Vision {
    private final WebcamName frontCamera;
    private final WebcamName backCamera;

    public VisionPortal spikeMarkPortal = null;
    public VisionPortal aprilTagPortal = null;

    SpikeLocationDetectionProcessor spikeMarkDetectionProcessor = null;
    AprilTagProcessor aprilTagProcessor = null;

    public Vision(HardwareMap hardwareMap) {
        frontCamera = hardwareMap.get(WebcamName.class, "Webcam 2");
        backCamera = hardwareMap.get(WebcamName.class, "Webcam 1");
    }

    public void startProcessors() {
        int[] portalsList = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.HORIZONTAL);

        spikeMarkDetectionProcessor = new SpikeLocationDetectionProcessor();
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setLensIntrinsics(913.562, 913.562, 650.769, 346.753)
                .setDrawCubeProjection(true)
                .setDrawAxes(true)
                .build();

//        spikeMarkPortal = VisionPortal.easyCreateWithDefaults(frontCamera, spikeMarkDetectionProcessor, aprilTagProcessor);

        VisionPortal.Builder builder1 = new VisionPortal.Builder();
        builder1.setCameraResolution(new Size(1280, 720));
        builder1.setCamera(frontCamera);
        builder1.addProcessor(spikeMarkDetectionProcessor);
        builder1.setLiveViewContainerId(portalsList[0]);
        spikeMarkPortal = builder1.build();

        VisionPortal.Builder builder2 = new VisionPortal.Builder();
        builder2.setCameraResolution(new Size(1280, 720));
        builder2.setCamera(backCamera);
        builder2.addProcessor(aprilTagProcessor);
        builder2.setLiveViewContainerId(portalsList[1]);
        aprilTagPortal = builder2.build();
    }

    public void setActiveProcessor(VisionProcessor processor) {
        if (processor == VisionProcessor.APRIL_TAG_DETECTION) {
            aprilTagPortal.resumeStreaming();
            spikeMarkPortal.stopStreaming();
        } else {
            spikeMarkPortal.resumeStreaming();
            aprilTagPortal.stopStreaming();
        }
    }

    public BoardPosition getSpikeLocation() {
        if (spikeMarkDetectionProcessor != null) return spikeMarkDetectionProcessor.location;
        return null;
    }

    public void setColor(AutoColor color) {
        if (spikeMarkDetectionProcessor != null) spikeMarkDetectionProcessor.setColor(color);
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

    public boolean isBackCameraWorking() {
        return !aprilTagProcessor.getDetections().isEmpty();
    }

    public AprilTagProcessor getAprilTagProcessor() {
        return aprilTagProcessor;
    }

    public void close() {
        if (spikeMarkPortal != null) spikeMarkPortal.close();
        if (aprilTagPortal != null) aprilTagPortal.close();
    }
}
