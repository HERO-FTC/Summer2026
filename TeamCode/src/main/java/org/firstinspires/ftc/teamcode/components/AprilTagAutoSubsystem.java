
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

    private final intakeSubsystem intake;
    private final ShooterTurretSubsystem shooter;
    private final RobotInit robot;

    private final boolean isBlue;

    private final ElapsedTime driveTimer = new ElapsedTime();
    private final ElapsedTime feedTimer = new ElapsedTime();
    private final ElapsedTime cooldownTimer = new ElapsedTime();

    private static final double DRIVE_POWER = 0.4;
    private static final double DRIVE_TIME = 0.1;

    private static final double BALL_FEED_TIME = 0.4;
    private static final double BALL_INTERVAL = 0.7;

    private boolean driveFinished = false;
    private boolean feeding = false;
    private boolean hasFed = false;
    private boolean autoStarted = false;
    private boolean shootingEnabled = false;
    private boolean visionClosed = false;

    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;

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

    public void startAuto() {

        shooter.startFlywheel();

        intake.runOuterAndMiddle(1.0);
        intake.runInner(0.0);

        shootingEnabled = false;
        hasFed = false;
        feeding = false;
        driveFinished = false;

        driveTimer.reset();
        autoStarted = true;
    }

    public void setShootingEnabled(boolean enabled) {

        shootingEnabled = enabled;

        if (!enabled) {
            intake.runInner(0.0);
            feeding = false;
        }

        if (enabled) {
            hasFed = false;
            cooldownTimer.reset();
        }
    }

    public void update() {

        if (!autoStarted) {
            return;
        }

        intake.runOuterAndMiddle(1.0);

        if (!driveFinished) {

            if (driveTimer.seconds() < DRIVE_TIME) {

                robot.frontLeft.setPower(DRIVE_POWER);
                robot.frontRight.setPower(DRIVE_POWER);
                robot.backLeft.setPower(DRIVE_POWER);
                robot.backRight.setPower(DRIVE_POWER);

            } else {

                stopDrive();
                driveFinished = true;
            }

            return;
        }

        if (!shootingEnabled) {
            return;
        }

        boolean targetVisible = isCorrectAllianceTargetVisible();

        if (feeding) {

            if (!targetVisible ||
                    feedTimer.seconds() >= BALL_FEED_TIME) {

                intake.runInner(0.0);
                feeding = false;
            }

            return;
        }

        if (targetVisible &&
                (!hasFed || cooldownTimer.seconds() >= BALL_INTERVAL)) {

            intake.runInner(1.0);

            feeding = true;
            hasFed = true;

            feedTimer.reset();
            cooldownTimer.reset();
        }
    }

    private boolean isCorrectAllianceTargetVisible() {

        List<AprilTagDetection> detections =
                aprilTag.getDetections();

        for (AprilTagDetection detection : detections) {

            if (detection instanceof AprilTagClusterDetection) {

                AprilTagClusterDetection cluster =
                        (AprilTagClusterDetection) detection;

                if (cluster.metadata != null &&
                        cluster.metadata.name != null &&
                        detection.ftcPose != null) {

                    String name = cluster.metadata.name;

                    boolean correctAlliance =
                            isBlue
                                    ? name.startsWith("B")
                                    : name.startsWith("R");

                    if (correctAlliance &&
                            Math.abs(detection.ftcPose.roll) < 90) {

                        return true;
                    }
                }
            }
        }

        return false;
    }

    private void stopDrive() {

        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);
    }

    public boolean isDriveFinished() {
        return driveFinished;
    }

    public void stop() {

        stopDrive();

        intake.stop();
        shooter.stop();

        feeding = false;
        shootingEnabled = false;
        autoStarted = false;

        if (!visionClosed) {
            visionPortal.close();
            visionClosed = true;
        }
    }
}
