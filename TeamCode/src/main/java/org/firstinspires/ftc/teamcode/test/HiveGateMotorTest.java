package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.components.HiveGate;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

@TeleOp(name = "Hive Gate Motor Test", group = "Tests")
public class HiveGateMotorTest extends LinearOpMode {

    // low power on purpose, this is just a test
    private static final double TEST_POWER = 0.4;

    @Override
    public void runOpMode() {

        // ---- motor: change "o1" to whatever your motor is called in the config ----
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "o1");
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // ---- camera: inches and degrees, because HiveGate assumes those units ----
        AprilTagProcessor aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        VisionPortal portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();
        // add the other CELL's IDs from Figure 9-17 so it keeps working after the HIVE tips
        Set<Integer> myCells = new HashSet<>(Arrays.asList(38, 39, 40, 41));
        HiveGate hive = new HiveGate(aprilTag, myCells, telemetry);

        telemetry.addLine("Ready. Point the camera at the HIVE and press START.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // let HiveGate look at the newest camera frame
            hive.update();

            // green light = spin, anything else = stop
            // (isTipping is an extra safety so we cut power the instant it moves)
            if (hive.isReady() && !hive.isTipping()) {
                motor.setPower(TEST_POWER);
            } else {
                motor.setPower(0);
            }

            // everything you need to tune the thresholds
            hive.addTelemetry();
            telemetry.addData("Motor power", "%.2f", motor.getPower());
            telemetry.update();
        }

        // clean up so the motor definitely stops and the camera closes
        motor.setPower(0);
        portal.close();
    }
}