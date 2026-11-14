package org.firstinspires.ftc.teamcode.ppturret;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

/**
 * Pinpoint (via Pedro Pathing) position + heading telemetry, plus turret heading telemetry.
 * Nothing here moves the turret or the robot. Push the robot and turn the turret by hand.
 *
 * ROBOT X / Y:
 *   Origin (0,0) = where the robot starts. +Y = toward the goals, +X = to the robot's starting right.
 *
 * ROBOT HEADING and TURRET HEADING (same convention for both):
 *   0 at the start (pointing forward). 90 = a quarter turn to the right, -90 = a quarter turn
 *   to the left. Range is -180..+180, so a half turn reads 180.
 *
 * Gamepad 1 A: reset robot position to (0,0) and robot heading to 0.
 * Gamepad 1 B: set the turret's current direction as 0 (do this with the turret pointing forward).
 *              The turret is also set to 0 automatically when the OpMode starts.
 */
@TeleOp(name = "PPTurretMain")
public class ppturretmain extends LinearOpMode {

    // ---------------- Pedro start pose (internal, leave as is) ----------------
    // Pedro math frame: 0 deg = +X, 90 deg = +Y, counter-clockwise positive.
    private static final double START_HEADING_DEG = 90.0;

    // ---------------- Axis settings ----------------
    // Flip one of these to the other sign if that value reads backwards.
    private static final double X_SIGN = -1.0;
    private static final double Y_SIGN = -1.0;
    private static final double HEADING_SIGN = -1.0;

    // ---------------- Turret feedback settings ----------------
    // Analog input name of the Axon Max feedback wire (must match your robot configuration).
    private static final String TURRET_ENCODER_NAME = "turretEncoder";

    // The Control Hub analog ports read 0-3.3V. Axon feedback covers 360 degrees of servo rotation.
    private static final double ANALOG_MAX_VOLTS = 3.3;
    private static final double FEEDBACK_RANGE_DEG = 360.0;

    // Servo rotations per 1 rotation of the turret.
    // 1.0 = servo horn directly on the turret. If a 60T servo gear drives a 120T turret gear, use 2.0.
    private static final double SERVO_TO_TURRET_RATIO = 1.0;

    // -1.0 or 1.0. Flip this if turning the turret to the right reads negative.
    private static final double TURRET_DIRECTION = -1.0;

    private Follower follower;
    private AnalogInput turretEncoder;

    @Override
    public void runOpMode() {

        Pose startPose = new Pose(0.0, 0.0, Math.toRadians(START_HEADING_DEG));

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        turretEncoder = hardwareMap.get(AnalogInput.class, TURRET_ENCODER_NAME);

        boolean previousA = false;
        boolean previousB = false;

        telemetry.addLine("Ready");
        telemetry.addLine("Gamepad 1 A: reset robot position and heading");
        telemetry.addLine("Gamepad 1 B: set turret forward = 0");
        telemetry.update();

        waitForStart();

        // The turret's starting direction counts as 0.
        double turretZeroDeg = readFeedbackDeg();

        while (opModeIsActive()) {

            // Updates the Pinpoint localizer.
            follower.update();

            // Reset robot pose (edge detected so it only fires once per press).
            boolean aPressed = gamepad1.a;
            if (aPressed && !previousA) {
                follower.setPose(startPose);
            }
            previousA = aPressed;

            // Zero the turret at its current direction.
            boolean bPressed = gamepad1.b;
            if (bPressed && !previousB) {
                turretZeroDeg = readFeedbackDeg();
            }
            previousB = bPressed;

            // ---------------- Robot pose ----------------
            Pose pose = follower.getPose();
            double robotX = X_SIGN * pose.getX();
            double robotY = Y_SIGN * pose.getY();

            // Heading relative to the start: right is positive, left is negative, -180..180.
            double robotHeadingDeg = normalize180(
                    HEADING_SIGN * (Math.toDegrees(pose.getHeading()) - START_HEADING_DEG));

            // ---------------- Turret heading ----------------
            // Same convention as the robot heading: right is positive, left is negative, -180..180.
            double turretHeadingDeg = normalize180(
                    (readFeedbackDeg() - turretZeroDeg) * TURRET_DIRECTION / SERVO_TO_TURRET_RATIO);

            // ---------------- Telemetry ----------------
            telemetry.addData("Robot X (in)", "%.2f", robotX);
            telemetry.addData("Robot Y (in)", "%.2f", robotY);
            telemetry.addData("Robot Heading (deg)", "%.1f", robotHeadingDeg);
            telemetry.addData("Turret Heading (deg)", "%.1f", turretHeadingDeg);
            telemetry.update();
        }
    }

    // Servo-side angle from the Axon feedback wire, 0..360.
    private double readFeedbackDeg() {
        return (turretEncoder.getVoltage() / ANALOG_MAX_VOLTS) * FEEDBACK_RANGE_DEG;
    }

    // Wraps an angle to 0..360.
    private static double normalize360(double deg) {
        deg = deg % 360.0;
        if (deg < 0) deg += 360.0;
        return deg;
    }

    // Wraps an angle to -180..180.
    private static double normalize180(double deg) {
        deg = normalize360(deg);
        if (deg > 180.0) deg -= 360.0;
        return deg;
    }
}