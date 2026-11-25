//package org.firstinspires.ftc.teamcode.components;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//
//import org.firstinspires.ftc.vision.VisionPortal;
//import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
//import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
//import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
//
//import java.util.List;
//import java.util.ArrayList;
//import java.util.Collections;
//
//public class aprilTagCameraDetectT {
//
//    // Camera and AprilTag objects
//    private VisionPortal visionPortal;
//    private AprilTagProcessor aprilTagProcessor;
//
//    // Initialize the camera
//    public void init(WebcamName webcam) {
//
//        // Create the AprilTag processor
//        aprilTagProcessor = new AprilTagProcessor.Builder()
//                .build();
//
//        // Create and start the camera portal
//        visionPortal = new VisionPortal.Builder()
//                .setCamera(webcam)
//                .addProcessor(aprilTagProcessor)
//                .build();
//    }
//
//    // Get all AprilTag detections
//    public List<AprilTagDetection> getDetections() {
//
//        if (aprilTagProcessor == null) {
//            return Collections.emptyList();
//        }
//
//        return aprilTagProcessor.getDetections();
//    }
//
//    // Get only AprilTag cluster detections
//    public List<AprilTagClusterDetection> getClusterDetections() {
//
//        List<AprilTagClusterDetection> clusters = new ArrayList<>();
//
//        // Get all current detections
//        List<AprilTagDetection> detections = getDetections();
//
//        // Examine every detection
//        for (AprilTagDetection detection : detections) {
//
//            // Check whether this detection is a cluster
//            if (detection instanceof AprilTagClusterDetection) {
//
//                // Convert it into a cluster detection
//                AprilTagClusterDetection cluster =
//                        (AprilTagClusterDetection) detection;
//
//                // Add it to our cluster list
//                clusters.add(cluster);
//            }
//        }
//
//        return clusters;
//    }
//
//    // Check whether at least one cluster is visible
//    public boolean hasCluster() {
//        return !getClusterDetections().isEmpty();
//    }
//
//    // Check whether the camera is streaming
//    public boolean isStreaming() {
//
//        return visionPortal != null &&
//                visionPortal.getCameraState() ==
//                        VisionPortal.CameraState.STREAMING;
//    }
//
//    // Enable or disable AprilTag processing
//    public void setDetectionEnabled(boolean enabled) {
//
//        if (visionPortal != null && aprilTagProcessor != null) {
//            visionPortal.setProcessorEnabled(
//                    aprilTagProcessor, enabled
//            );
//        }
//    }
//
//    // Shut down the camera
//    public void close() {
//
//        if (visionPortal != null) {
//            visionPortal.close();
//            visionPortal = null;
//        }
//
//        aprilTagProcessor = null;
//    }
//}


package org.firstinspires.ftc.teamcode.components;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

import java.util.List;
import java.util.ArrayList;
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

    // Get all AprilTag detections
    public List<AprilTagDetection> getDetections() {

        if (aprilTagProcessor == null) {
            return Collections.emptyList();
        }

        return aprilTagProcessor.getDetections();
    }

    // Get only AprilTag cluster detections
    public List<AprilTagClusterDetection> getClusterDetections() {

        List<AprilTagClusterDetection> clusters = new ArrayList<>();

        // Get all current detections
        List<AprilTagDetection> detections = getDetections();

        // Examine every detection
        for (AprilTagDetection detection : detections) {

            // Check whether this detection is a cluster
            if (detection instanceof AprilTagClusterDetection) {

                // Convert it into a cluster detection
                AprilTagClusterDetection cluster =
                        (AprilTagClusterDetection) detection;

                // Add it to our cluster list
                clusters.add(cluster);
            }
        }

        return clusters;
    }

    // Check whether at least one cluster is visible
    public boolean hasCluster() {
        return !getClusterDetections().isEmpty();
    }

    // Check whether the cluster is currently visible
    public boolean isClusterVisible() {
        return hasCluster();
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
