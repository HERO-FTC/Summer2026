
package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.RobotInit;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;

import java.util.List;

public class AprilTagAutoSubsystem {

    // Hardware and subsystems
    private final RobotInit robot;
    private final intakeSubsystem intake;
    private final ShooterTurretSubsystem shooter;

    // Alliance
    private final boolean isBlue;

    // Timers
    private final ElapsedTime driveTimer = new ElapsedTime();
    private final ElapsedTime flywheelTimer = new ElapsedTime();
    private final ElapsedTime feedTimer = new ElapsedTime();
    private final ElapsedTime cooldownTimer = new ElapsedTime();

    // Drive settings
    private static final double DRIVE_POWER = 0.4;
    private static final double DRIVE_TIME = 0.5;

    // Flywheel delay
    private static final double FLYWHEEL_DELAY = 4.0;

    // Shooting settings
    private static final double BALL_FEED_TIME = 0.4;
    private static final double BALL_INTERVAL = 0.7;

    // State variables
    private boolean driveFinished = false;
    private boolean shootingEnabled = false;
    private boolean flywheelTimerStarted = false;
    private boolean flywheelReady = false;
    private boolean hasFed = false;
    private boolean feeding = false;

    // AprilTag vision
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    public AprilTagAutoSubsystem(
            HardwareMap hardwareMap,
            intakeSubsystem intake,
            ShooterTurretSubsystem shooter,
            boolean isBlue,
            RobotInit robot) {

        this.intake = intake;
        this.shooter = shooter;
        this.isBlue = isBlue;
        this.robot = robot;

        aprilTag = new AprilTagProcessor.Builder()
                .setTagLibrary(
                        AprilTagGameDatabase.getCurrentGameTagLibrary()
                )
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(
                        hardwareMap.get(WebcamName.class, "Webcam 1")
                )
                .addProcessor(aprilTag)
                .build();
    }

    // Starts autonomous
    public void startAuto() {

        stopDrive();

        shooter.startFlywheel();

        intake.runOuterAndMiddle(1.0);
        intake.runInner(0.0);

        driveFinished = false;
        shootingEnabled = false;

        flywheelTimerStarted = false;
        flywheelReady = false;

        hasFed = false;
        feeding = false;

        driveTimer.reset();
    }

    // Main autonomous loop
    public void update() {

        // Keep outer and middle intake running
        intake.runOuterAndMiddle(1.0);

        // STEP 1: Drive forward using only the two back wheels
        if (!driveFinished) {

            if (driveTimer.seconds() < DRIVE_TIME) {

                robot.backLeft.setPower(DRIVE_POWER);
                robot.backRight.setPower(DRIVE_POWER);

            } else {

                stopDrive();
                driveFinished = true;

                // Start the flywheel delay
                flywheelTimer.reset();
                flywheelTimerStarted = true;
            }

            return;
        }

        // STEP 2: Wait 4 seconds for flywheel spin-up
        if (!flywheelReady) {

            if (flywheelTimerStarted &&
                    flywheelTimer.seconds() >= FLYWHEEL_DELAY) {

                flywheelReady = true;

            } else {
                return;
            }
        }

        // STEP 3: Wait until shooting is enabled
        if (!shootingEnabled) {
            return;
        }

        // STEP 4: Check for the correct alliance AprilTag
        boolean targetVisible = isCorrectAllianceTargetVisible();

        // STEP 5: Feed while the target is visible
        if (feeding) {

            if (!targetVisible ||
                    feedTimer.seconds() >= BALL_FEED_TIME) {

                intake.runInner(0.0);
                feeding = false;
                cooldownTimer.reset();

            } else {

                intake.runInner(1.0);
            }

        } else if (targetVisible &&
                (!hasFed || cooldownTimer.seconds() >= BALL_INTERVAL)) {

            intake.runInner(1.0);

            feeding = true;
            hasFed = true;

            feedTimer.reset();
        }
    }

    // Detect the correct alliance AprilTag cluster
    private boolean isCorrectAllianceTargetVisible() {

        List<AprilTagDetection> detections = aprilTag.getDetections();

        for (AprilTagDetection detection : detections) {

            if (detection instanceof AprilTagClusterDetection) {

                AprilTagClusterDetection cluster =
                        (AprilTagClusterDetection) detection;

                if (cluster.metadata != null &&
                        cluster.metadata.name != null) {

                    String name = cluster.metadata.name;

                    boolean correctAlliance = isBlue
                            ? name.startsWith("B")
                            : name.startsWith("R");

                    if (correctAlliance) {
                        return true;
                    }
                }
            }
        }

        return false;
    }

    // Stop all drivetrain motors
    private void stopDrive() {

        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
    }

    public boolean isDriveFinished() {
        return driveFinished;
    }

    public void setShootingEnabled(boolean enabled) {
        shootingEnabled = enabled;
    }

    // Stop autonomous
    public void stop() {

        stopDrive();

        intake.stop();
        shooter.stop();

        feeding = false;
        shootingEnabled = false;

        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
