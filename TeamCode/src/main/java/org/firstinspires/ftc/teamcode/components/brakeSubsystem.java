package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Brake + Flicker Test", group = "Testing")

public class brakeSubsystem extends LinearOpMode {

    private Servo brakeServo;
    private Servo flickerServo;

    // Brake servo positions
    private static final double BRAKE_IN = 0.47;
    private static final double BRAKE_OUT = 0.07;

    // Flicker servo positions
    private static final double FLICKER_ON = 0.5;
    private static final double FLICKER_OFF = 0.0;

    // Track servo states
    private boolean brakeIsIn = false;
    private boolean flickerIsOn = false;

    @Override
    public void runOpMode() {

        // Connect servos to hardware configuration
        brakeServo = hardwareMap.get(Servo.class, "brakeServo");
        flickerServo = hardwareMap.get(Servo.class, "flickerServo");

        // Initial servo positions
        brakeServo.setPosition(BRAKE_OUT);
        flickerServo.setPosition(FLICKER_OFF);

        telemetry.addLine("Brake + Flicker Test Ready");
        telemetry.addLine("A = Toggle Brake");
        telemetry.addLine("B = Toggle Flicker");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // A BUTTON: Toggle brake
            if (gamepad1.aWasPressed()) {

                brakeIsIn = !brakeIsIn;

                if (brakeIsIn) {
                    brakeServo.setPosition(BRAKE_IN);
                } else {
                    brakeServo.setPosition(BRAKE_OUT);
                }
            }

            // B BUTTON: Toggle flicker
            if (gamepad1.bWasPressed()) {

                flickerIsOn = !flickerIsOn;

                if (flickerIsOn) {
                    flickerServo.setPosition(FLICKER_ON);
                } else {
                    flickerServo.setPosition(FLICKER_OFF);
                }
            }

            // Display current positions
            telemetry.addData("Brake", brakeIsIn ? "IN" : "OUT");
            telemetry.addData("Flicker", flickerIsOn ? "ON" : "OFF");
            telemetry.update();
        }
    }
}
