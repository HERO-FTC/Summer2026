
package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "Color Sensor Axon MAX Test")
public class ColorHoodTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        ColorSensor colorSensor =
                hardwareMap.get(ColorSensor.class, "colorSensor");

        Servo hoodServo =
                hardwareMap.get(Servo.class, "hoodServo");

        final double RED_POSITION = 0.20;
        final double BLUE_POSITION = 0.20;
        final double YELLOW_POSITION = 0.80;

        final int CONFIRMATION_COUNT = 2;

        double lastPosition = -1;

        String currentColor = "Unknown";
        String candidateColor = "Unknown";

        int candidateCount = 0;

        waitForStart();

        while (opModeIsActive()) {

            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();

            String detectedColor = "Unknown";

            if (red > green * 1.3 && red > blue * 1.3) {
                detectedColor = "RED";
            }
            else if (blue > red * 1.3 && blue > green * 1.3) {
                detectedColor = "BLUE";
            }
            else if (red > 0 && green > 0
                    && red > blue * 1.5
                    && green > blue * 1.5) {
                detectedColor = "YELLOW";
            }

            if (!detectedColor.equals("Unknown")) {

                if (detectedColor.equals(candidateColor)) {
                    candidateCount++;
                } else {
                    candidateColor = detectedColor;
                    candidateCount = 1;
                }

                if (candidateCount >= CONFIRMATION_COUNT
                        && !candidateColor.equals(currentColor)) {

                    currentColor = candidateColor;

                    double targetPosition;

                    switch (currentColor) {
                        case "RED":
                            targetPosition = RED_POSITION;
                            break;

                        case "BLUE":
                            targetPosition = BLUE_POSITION;
                            break;

                        case "YELLOW":
                            targetPosition = YELLOW_POSITION;
                            break;

                        default:
                            targetPosition = lastPosition;
                            break;
                    }

                    hoodServo.setPosition(targetPosition);
                    lastPosition = targetPosition;
                }
            }

            telemetry.addData("Detected", detectedColor);
            telemetry.addData("Confirmed", currentColor);
            telemetry.addData("Candidate Count", candidateCount);
            telemetry.addData("Servo Position", lastPosition);
            telemetry.addData("Red", red);
            telemetry.addData("Green", green);
            telemetry.addData("Blue", blue);
            telemetry.update();
        }
    }
}
