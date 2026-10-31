
package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;

    private final AprilTagProcessor aprilTag;
    private final VisionPortal visionPortal;

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

    public AprilTagAutoSubsystem(
            HardwareMap hardwareMap,
            intakeSubsystem intake,
            ShooterTurretSubsystem shooter) {

        this.intake = intake;
        this.shooter = shooter;

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

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

        // Keep outer and middle intake running.
        intake.runOuterAndMiddle(1.0);

        // Inner intake starts off.
        intake.runInner(0.0);

        shootingEnabled = false;
        hasFed = false;
        feeding = false;

        driveFinished = false;

        // Start the 100ms drive timer.
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

        // Outer and middle intake ALWAYS run.
        intake.runOuterAndMiddle(1.0);

        // Drive forward at 40% for 100ms.
        if (!driveFinished) {

            if (driveTimer.seconds() < DRIVE_TIME) {

                frontLeft.setPower(DRIVE_POWER);
                frontRight.setPower(DRIVE_POWER);
                backLeft.setPower(DRIVE_POWER);
                backRight.setPower(DRIVE_POWER);

            } else {

                stopDrive();
                driveFinished = true;
            }

            return;
        }

        // Wait until shooting is enabled.
        if (!shootingEnabled) {
            return;
        }

        boolean targetVisible = isBlueScorableCellVisible();

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

    private boolean isBlueScorableCellVisible() {

        List<AprilTagDetection> detections =
                aprilTag.getDetections();

        for (AprilTagDetection detection : detections) {

            if (detection instanceof AprilTagClusterDetection) {

                AprilTagClusterDetection cluster =
                        (AprilTagClusterDetection) detection;

                if (cluster.metadata != null &&
                        cluster.metadata.name != null &&
                        cluster.metadata.name.startsWith("B") &&
                        detection.ftcPose != null &&
                        Math.abs(detection.ftcPose.roll) < 90) {

                    return true;
                }
            }
        }

        return false;
    }

    private void stopDrive() {

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
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
    public boolean isDriveFinished() {
        return driveFinished;
    }
}
