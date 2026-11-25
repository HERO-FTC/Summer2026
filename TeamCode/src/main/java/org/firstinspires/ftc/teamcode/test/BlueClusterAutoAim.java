
package org.firstinspires.ftc.teamcode.localization;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;

import java.util.List;

@TeleOp(name = "Blue Cluster Auto Aim", group = "Turret")
public class BlueClusterAutoAim extends OpMode {

    private Servo turretServo;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Servo limits
    static final double SERVO_MIN = 0.05;
    static final double SERVO_MAX = 0.95;
    static final double SERVO_CENTER = 0.50;

    // Tracking settings
    static final double KP = 0.001;
    static final double DEAD_BAND = 0.8;
    static final double MAX_STEP = 0.01;

    // Reverse if turret moves away from target
    static final double SERVO_DIRECTION = 1.0;

    private double servoPosition = SERVO_CENTER;

    @Override
    public void init() {

        turretServo = hardwareMap.get(
                Servo.class, "turretServo"
        );

        turretServo.setPosition(SERVO_CENTER);

        aprilTag = new AprilTagProcessor.Builder()
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(
                        WebcamName.class, "Webcam 1"
                ))
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("Blue Cluster Auto Aim Ready");
        telemetry.update();
    }

    @Override
    public void loop() {

        List<AprilTagDetection> detections =
                aprilTag.getDetections();

        AprilTagClusterDetection bestTarget = null;

        // Find an upright BLUE cluster
        for (AprilTagDetection detection : detections) {

            if (!(detection instanceof AprilTagClusterDetection)) {
                continue;
            }

            AprilTagClusterDetection cluster =
                    (AprilTagClusterDetection) detection;

            if (cluster.metadata == null ||
                    cluster.ftcPose == null) {
                continue;
            }

            // Only blue clusters
            if (!cluster.metadata.name.startsWith("B")) {
                continue;
            }

            // Only upright/scorable cells
            if (Math.abs(cluster.ftcPose.roll) >= 90) {
                continue;
            }

            // Select the closest valid blue cluster
            if (bestTarget == null ||
                    cluster.ftcPose.range <
                    bestTarget.ftcPose.range) {

                bestTarget = cluster;
            }
        }

        if (bestTarget != null) {

            double bearing = bestTarget.ftcPose.bearing;

            // Center the blue target
            if (Math.abs(bearing) > DEAD_BAND) {

                double adjustment = Range.clip(
                        bearing * KP,
                        -MAX_STEP,
                        MAX_STEP
                );

                servoPosition +=
                        SERVO_DIRECTION * adjustment;

                servoPosition = Range.clip(
                        servoPosition,
                        SERVO_MIN,
                        SERVO_MAX
                );

                turretServo.setPosition(servoPosition);
            }

            telemetry.addData("Target", bestTarget.metadata.name);
            telemetry.addData("Bearing", "%.2f", bearing);
            telemetry.addData("Roll", "%.2f",
                    bestTarget.ftcPose.roll);
            telemetry.addData("Range", "%.2f",
                    bestTarget.ftcPose.range);

        } else {

            // Hold position if no valid blue target is found
            telemetry.addLine("NO VALID BLUE TARGET");
        }

        telemetry.addData("Servo Position", "%.3f",
                servoPosition);

        telemetry.update();
    }

    @Override
    public void stop() {

        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
