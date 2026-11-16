package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.components.aprilTagCameraDetectT;

import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "Camera Test", group = "Testing")
public class CameraTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Create camera subsystem
        aprilTagCameraDetectT camera = new aprilTagCameraDetectT();

        // Retrieve webcam
        WebcamName webcam = hardwareMap.get(
                WebcamName.class, "Webcam 1"
        );

        // Initialize camera
        camera.init(webcam);

        telemetry.addLine("Camera initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Display camera status
            telemetry.addData(
                    "Camera Status",
                    camera.isStreaming() ? "STREAMING" : "NOT STREAMING"
            );

            // Get detected clusters
            List<AprilTagClusterDetection> clusters =
                    camera.getClusterDetections();

            // Display number of detected clusters
            telemetry.addData("Clusters Detected", clusters.size());

            if (clusters.isEmpty()) {

                telemetry.addLine("No BIOBUZZ clusters detected");

            } else {

                // Display information for every detected cluster
                for (int i = 0; i < clusters.size(); i++) {

                    AprilTagClusterDetection cluster = clusters.get(i);

                    telemetry.addData(
                            "Cluster " + (i + 1),
                            cluster.metadata.name
                    );

                    telemetry.addData(
                            "Percent Found",
                            "%d%%",
                            cluster.percentClusterFound
                    );

                    // Display target position
                    if (cluster.ftcPose != null) {

                        telemetry.addData(
                                "X (Right)",
                                "%.1f inches",
                                cluster.ftcPose.x
                        );

                        telemetry.addData(
                                "Y (Forward)",
                                "%.1f inches",
                                cluster.ftcPose.y
                        );

                        telemetry.addData(
                                "Z (Up)",
                                "%.1f inches",
                                cluster.ftcPose.z
                        );

                        telemetry.addData(
                                "Bearing",
                                "%.1f degrees",
                                cluster.ftcPose.bearing
                        );

                        telemetry.addData(
                                "Roll",
                                "%.1f degrees",
                                cluster.ftcPose.roll
                        );
                    }
                }
            }

            telemetry.update();

            sleep(100);
        }

        // Shut down camera
        camera.close();
    }
}