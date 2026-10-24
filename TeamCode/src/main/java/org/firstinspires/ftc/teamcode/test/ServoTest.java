
package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test", group = "Components")
public class ServoTest extends LinearOpMode {

    private Servo servo;

    private double position = 0.5;
    private double minPosition = 0.0;
    private double maxPosition = 1.0;

    private final double step = 0.01;

    @Override
    public void runOpMode() {

        servo = hardwareMap.get(Servo.class, "flowerServo");

        servo.setPosition(position);

        telemetry.addLine("=== SERVO TEST ===");
        telemetry.addLine("Dpad Right: Increase position");
        telemetry.addLine("Dpad Left: Decrease position");
        telemetry.addLine("A: Save minimum");
        telemetry.addLine("B: Save maximum");
        telemetry.addLine("X: Go to minimum");
        telemetry.addLine("Y: Go to maximum");
        telemetry.addLine("Start: Reset to 0.50");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Increase position
            if (gamepad1.dpad_right) {
                position += step;
            }

            // Decrease position
            if (gamepad1.dpad_left) {
                position -= step;
            }

            // Keep position between 0 and 1
            position = Math.max(0.0, Math.min(1.0, position));

            // Save minimum and maximum
            if (gamepad1.a) {
                minPosition = position;
            }

            if (gamepad1.b) {
                maxPosition = position;
            }

            // Go to saved positions
            if (gamepad1.x) {
                position = minPosition;
            }

            if (gamepad1.y) {
                position = maxPosition;
            }

            // Reset to center
            if (gamepad1.start) {
                position = 0.5;
            }

            servo.setPosition(position);

            telemetry.addData("Current Position", "%.2f", position);
            telemetry.addData("Minimum Saved", "%.2f", minPosition);
            telemetry.addData("Maximum Saved", "%.2f", maxPosition);
            telemetry.addData("Step Size", "%.2f", step);

            telemetry.update();

            sleep(100);
        }
    }
}
