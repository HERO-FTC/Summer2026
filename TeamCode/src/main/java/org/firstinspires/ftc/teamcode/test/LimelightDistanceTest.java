package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Limelighttargeting;


@TeleOp(name = "Limelight Distance Test", group = "Tests")
public class LimelightDistanceTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Limelighttargeting ll = new Limelighttargeting(hardwareMap, "limelight");
        ll.setPipeline(0);
        ll.configure(10.0, 25.0, 29.5); // camera height, pitch, tag height (inches/deg) - MEASURE THESE

        double knownDistance = 48.0;
        boolean upPrev = false, downPrev = false, rightPrev = false, leftPrev = false;
        boolean aPrev = false, bPrev = false, xPrev = false;

        ll.start();
        telemetry.addLine("Ready. Press START.");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            // Edge detection so each press = one step
            if (gamepad1.dpad_up && !upPrev)       ll.setCameraPitchDeg(ll.getCameraPitchDeg() + 0.5);
            if (gamepad1.dpad_down && !downPrev)   ll.setCameraPitchDeg(ll.getCameraPitchDeg() - 0.5);
            if (gamepad1.dpad_right && !rightPrev) knownDistance += 1.0;
            if (gamepad1.dpad_left && !leftPrev)   knownDistance -= 1.0;
            if (gamepad1.b && !bPrev)              ll.setCameraHeightIn(ll.getCameraHeightIn() + 0.25);
            if (gamepad1.x && !xPrev)              ll.setCameraHeightIn(ll.getCameraHeightIn() - 0.25);

            boolean seen = ll.update();

            if (gamepad1.a && !aPrev && seen) {
                ll.setCameraPitchDeg(ll.calibratePitchDeg(knownDistance));
            }

            upPrev = gamepad1.dpad_up;     downPrev = gamepad1.dpad_down;
            rightPrev = gamepad1.dpad_right; leftPrev = gamepad1.dpad_left;
            aPrev = gamepad1.a;            bPrev = gamepad1.b;  xPrev = gamepad1.x;

            telemetry.addData("Target visible", seen);
            telemetry.addData("tx (deg)", "%.2f", ll.getTx());
            telemetry.addData("ty (deg)", "%.2f", ll.getTy());
            telemetry.addData("Distance (in)", "%.2f", ll.getDistanceIn());
            telemetry.addData("Line of sight (in)", "%.2f", ll.getLineOfSightIn());
            telemetry.addLine("--- config ---");
            telemetry.addData("Camera pitch (deg)", "%.2f", ll.getCameraPitchDeg());
            telemetry.addData("Camera height (in)", "%.2f", ll.getCameraHeightIn());
            telemetry.addData("Tag height (in)", "%.2f", ll.getTagHeightIn());
            telemetry.addData("Known distance (in)", "%.1f", knownDistance);
            telemetry.update();
        }

        ll.stop();
    }
}