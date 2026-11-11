package org.firstinspires.ftc.teamcode.components;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;
import java.util.Collections;

public class aprilTagCameraDetectT {

    // Camera and AprilTag objects
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    // Initialize the camera
    public void init(WebcamName webcam) {

        // Create the AprilTag processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .build();

        // Create and start the camera portal
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(aprilTagProcessor)
                .build();
    }

    // Get the latest AprilTag detections
    public List<AprilTagDetection> getDetections() {

        if (aprilTagProcessor == null) {
            return Collections.emptyList();
        }

        return aprilTagProcessor.getDetections();
    }

    // Check whether the camera is streaming
    public boolean isStreaming() {

        return visionPortal != null &&
                visionPortal.getCameraState() ==
                        VisionPortal.CameraState.STREAMING;
    }

    // Enable or disable AprilTag processing
    public void setDetectionEnabled(boolean enabled) {

        if (visionPortal != null && aprilTagProcessor != null) {
            visionPortal.setProcessorEnabled(
                    aprilTagProcessor, enabled
            );
        }
    }

    // Shut down the camera
    public void close() {

        if (visionPortal != null) {
            visionPortal.close();
            visionPortal = null;
        }

        aprilTagProcessor = null;
    }
}
