package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@TeleOp(name = "Tag Seen Motor Test", group = "Tests")
public class TagSeenMotorTest extends LinearOpMode {

    // low power on purpose, this is just a test
    private static final double TEST_POWER = 0.4;

    @Override
    public void runOpMode() {

        // change "o1" to whatever your motor is called in the config
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "o1");
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // default settings are fine since we only care whether a tag exists
        AprilTagProcessor aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        VisionPortal portal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);

        telemetry.addLine("Ready. Press START, then show the camera a tag.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // grab whatever tags the camera sees this frame
            List<AprilTagDetection> detections = aprilTag.getDetections();
            boolean seesTag = !detections.isEmpty();

            // tag in view = spin, nothing in view = stop
            motor.setPower(seesTag ? TEST_POWER : 0);

            telemetry.addData("Sees tag", seesTag);
            telemetry.addData("Tags in view", detections.size());
            for (AprilTagDetection d : detections) {
                telemetry.addData("Tag ID", d.id);
            }
            telemetry.addData("Motor power", "%.2f", motor.getPower());
            telemetry.update();
        }

        // make sure the motor stops and the camera closes
        motor.setPower(0);
        portal.close();
    }
}