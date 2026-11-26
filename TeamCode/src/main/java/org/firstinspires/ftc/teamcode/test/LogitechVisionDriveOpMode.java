package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

@Autonomous(name = "Logitech Vision Auto", group = "Autonomous")
public class LogitechVisionDriveOpMode extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private OpenCvWebcam webcam;
    private ColorBlobPipeline pipeline;

    private ElapsedTime lossTimer = new ElapsedTime();
    private ElapsedTime matchTimer = new ElapsedTime();

    private volatile boolean cameraReady = false;
    private volatile boolean cameraError = false;
    private volatile int cameraErrorCode = 0;

    @Override
    public void runOpMode() {

        // Initialize motors directly
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        int cameraMonitorViewId =
                hardwareMap.appContext.getResources().getIdentifier(
                        "cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName()
                );

        webcam = OpenCvCameraFactory.getInstance().createWebcam(
                hardwareMap.get(WebcamName.class, "Webcam 1"),
                cameraMonitorViewId
        );

        pipeline = new ColorBlobPipeline();

        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(5000);

        webcam.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {

                    @Override
                    public void onOpened() {
                        webcam.startStreaming(
                                1280,
                                720,
                                OpenCvCameraRotation.UPRIGHT,
                                OpenCvWebcam.StreamFormat.MJPEG
                        );

                        cameraReady = true;
                    }

                    @Override
                    public void onError(int errorCode) {
                        cameraError = true;
                        cameraErrorCode = errorCode;
                    }
                }
        );

        telemetry.addData("Status", "Initialized Autonomous");
        telemetry.addData("Resolution", "1280 x 720");
        telemetry.update();

        // Wait until START is pressed on the Driver Hub
        waitForStart();
        matchTimer.reset();
        lossTimer.reset();

        while (opModeIsActive()) {

            if (cameraError) {
                stopMotors();
                telemetry.addData("Camera Error", cameraErrorCode);
                telemetry.update();
                break;
            }

            if (!cameraReady) {
                stopMotors();
                telemetry.addData("Status", "Waiting for camera...");
                telemetry.update();
                continue;
            }

            double forwardPower = 0.0;
            double turnPower = 0.0;
            double strafePower = 0.0;

            if (pipeline.isDetected) {
                lossTimer.reset();

                double angle = pipeline.angleDeg;
                double distance = pipeline.distanceCm;

                telemetry.addData("Target", pipeline.targetColor);
                telemetry.addData("Objects", pipeline.objectCount);
                telemetry.addData("Angle", "%.1f deg", angle);
                telemetry.addData("Distance", "%.1f cm", distance);

                // P-Loop Steering to center the target horizontally
                double kP = 0.015;
                turnPower = angle * kP;
                turnPower = Math.max(-0.35, Math.min(0.35, turnPower));

                // Approach Logic: Drive forward until distance <= 5 cm
                if (distance > 5.0) {
                    forwardPower = 0.22;
                } else {
                    forwardPower = 0.0;
                    turnPower = 0.0;
                    stopMotors();
                    telemetry.addData("Status", "Target Reached (5 cm)!");
                    telemetry.update();

                    // Optional: Break out of autonomous loop once target is reached
                    // break;
                }

            } else {
                // If target is lost briefly, pause briefly before searching/stopping
                if (lossTimer.seconds() < 0.5) {
                    telemetry.addData("Status", "Target recently lost");
                } else {
                    telemetry.addData("Status", "Searching / Scanning");
                    forwardPower = 0.0;
                    turnPower = 0.15; // Slow spin to search for the ball if not in view
                }
            }

            // Apply mecanum drive powers
            double frontLeftPower = forwardPower + strafePower + turnPower;
            double frontRightPower = forwardPower - strafePower - turnPower;
            double backLeftPower = forwardPower - strafePower + turnPower;
            double backRightPower = forwardPower + strafePower - turnPower;

            double max = Math.max(
                    1.0,
                    Math.max(
                            Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                            Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
                    )
            );

            frontLeft.setPower(frontLeftPower / max);
            frontRight.setPower(frontRightPower / max);
            backLeft.setPower(backLeftPower / max);
            backRight.setPower(backRightPower / max);

            telemetry.update();
        }

        stopMotors();

        if (webcam != null && cameraReady) {
            webcam.stopStreaming();
            webcam.closeCameraDevice();
        }
    }

    private void stopMotors() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}