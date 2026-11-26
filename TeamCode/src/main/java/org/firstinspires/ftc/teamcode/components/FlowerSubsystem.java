package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.PwmControl;

@TeleOp(name = "Flower Servo Test", group = "Testing")
public class FlowerSubsystem extends LinearOpMode {

    private ServoImplEx rotationServo1;
    private ServoImplEx rotationServo2;
    private Servo clawServo;

    // ==========================================
    // ROTATION LIMITS
    // ==========================================

    private static final double SERVO1_MAX = 177.0;
    private static final double SERVO2_MAX = 30.0;

    private static final double SERVO2_FULL_RANGE = 280.0;

    // ==========================================
    // MOVEMENT SPEED
    // ==========================================

    private static final double SERVO1_STEP = 1.0;
    private static final double SERVO2_STEP = 0.5;

    // ==========================================
    // CLAW POSITIONS
    // ==========================================

    private static final double CLAW_OPEN = 0.2;
    private static final double CLAW_CLOSED = 0.8;

    // ==========================================
    // CURRENT POSITIONS
    // ==========================================

    private double servo1Angle = 88.5;
    private double servo2Angle = 15.0;

    private boolean clawOpen = false;
    private boolean previousA = false;

    @Override
    public void runOpMode() {

        // Initialize servos
        rotationServo1 = hardwareMap.get(
                ServoImplEx.class, "flowerRotate1");

        rotationServo2 = hardwareMap.get(
                ServoImplEx.class, "flowerRotate2");

        clawServo = hardwareMap.get(
                Servo.class, "flowerClaw");

        // Configure PWM ranges
        rotationServo1.setPwmRange(
                new PwmControl.PwmRange(600, 2400));

        rotationServo2.setPwmRange(
                new PwmControl.PwmRange(500, 2500));

        // Set starting positions
        updateServo1();
        updateServo2();

        clawServo.setPosition(CLAW_CLOSED);

        telemetry.addLine("FLOWER SERVO TEST");
        telemetry.addLine("Initialization complete");
        telemetry.addLine("Press PLAY to begin");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ==================================
            // SERVO 1 - PROTON
            // D-PAD UP / DOWN
            // ==================================

            double servo1Input =
                    (gamepad1.dpad_up ? 1 : 0) -
                            (gamepad1.dpad_down ? 1 : 0);

            servo1Angle += servo1Input * SERVO1_STEP;

            servo1Angle = Math.max(0,
                    Math.min(SERVO1_MAX, servo1Angle));

            updateServo1();

            // ==================================
            // SERVO 2 - ANDYMARK
            // D-PAD RIGHT / LEFT
            // ==================================

            double servo2Input =
                    (gamepad1.dpad_right ? 1 : 0) -
                            (gamepad1.dpad_left ? 1 : 0);

            servo2Angle += servo2Input * SERVO2_STEP;

            servo2Angle = Math.max(0,
                    Math.min(SERVO2_MAX, servo2Angle));

            updateServo2();

            // ==================================
            // CLAW SERVO
            // A BUTTON
            // ==================================

            if (gamepad1.a && !previousA) {

                clawOpen = !clawOpen;

                if (clawOpen) {
                    clawServo.setPosition(CLAW_OPEN);
                } else {
                    clawServo.setPosition(CLAW_CLOSED);
                }
            }

            previousA = gamepad1.a;

            // ==================================
            // TELEMETRY
            // ==================================

            telemetry.addLine("FLOWER SERVO TEST");

            telemetry.addData("PROTON Angle", servo1Angle);
            telemetry.addData("PROTON Position",
                    servo1Angle / SERVO1_MAX);

            telemetry.addData("AndyMark Angle", servo2Angle);
            telemetry.addData("AndyMark Position",
                    servo2Angle / SERVO2_FULL_RANGE);

            telemetry.addData("Claw",
                    clawOpen ? "OPEN" : "CLOSED");

            telemetry.update();
        }
    }

    // ==========================================
    // UPDATE SERVO 1
    // ==========================================

    private void updateServo1() {

        rotationServo1.setPosition(
                servo1Angle / SERVO1_MAX);
    }

    // ==========================================
    // UPDATE SERVO 2
    // ==========================================

    private void updateServo2() {

        rotationServo2.setPosition(
                servo2Angle / SERVO2_FULL_RANGE);
    }
}