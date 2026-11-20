package org.firstinspires.ftc.teamcode.test;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.components.aprilTagCameraDetectT;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Camera Test", group = "Testing")
public class CameraTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        aprilTagCameraDetectT camera = new aprilTagCameraDetectT();

        // Retrieve the webcam
        WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");

        // Initialize camera
        camera.init(webcam);

        telemetry.addLine("Camera initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData(
                    "Camera Status",
                    camera.isStreaming() ? "STREAMING" : "NOT STREAMING"
            );

            telemetry.update();

            sleep(100);
        }

        camera.close();
    }
}