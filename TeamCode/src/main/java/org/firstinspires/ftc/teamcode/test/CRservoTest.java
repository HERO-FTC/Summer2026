
package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "CRServo Test", group = "Components")
public class CRservoTest extends LinearOpMode {

    private CRServo servo;

    double servoPower = 0.50;
    static final double POWER_STEP = 0.05;

    boolean servoOn = false;
    boolean reversed = false;

    boolean lastA = false;
    boolean lastUp = false;
    boolean lastDown = false;
    boolean lastB = false;

    @Override
    public void runOpMode() {

        servo = hardwareMap.get(CRServo.class, "o1");

        servo.setDirection(CRServo.Direction.FORWARD);

        telemetry.addLine("CRServo Test");
        telemetry.addLine("A = Toggle servo");
        telemetry.addLine("D-pad Up/Down = Adjust power");
        telemetry.addLine("B = Reverse direction");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            boolean a = gamepad1.a;
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            boolean b = gamepad1.b;

            // Toggle servo on/off
            if (a && !lastA) {
                servoOn = !servoOn;
            }

            // Adjust power
            if (up && !lastUp) {
                servoPower += POWER_STEP;
            }

            if (down && !lastDown) {
                servoPower -= POWER_STEP;
            }

            // Reverse direction
            if (b && !lastB) {
                reversed = !reversed;
            }

            // Keep power between 0 and 1
            servoPower = Math.max(0.0, Math.min(1.0, servoPower));
            servoPower = Math.round(servoPower * 100.0) / 100.0;

            // Save button states
            lastA = a;
            lastUp = up;
            lastDown = down;
            lastB = b;

            // Set servo direction
            servo.setDirection(
                    reversed ? CRServo.Direction.REVERSE
                            : CRServo.Direction.FORWARD
            );

            // Apply power
            servo.setPower(servoOn ? servoPower : 0);

            // Telemetry
            telemetry.addData("Servo", servoOn ? "ON" : "OFF");
            telemetry.addData("Power", "%.0f%%", servoPower * 100);
            telemetry.addData("Direction", reversed ? "REVERSE" : "FORWARD");
            telemetry.update();
        }

        // Stop servo when OpMode ends
        servo.setPower(0);
    }
}
