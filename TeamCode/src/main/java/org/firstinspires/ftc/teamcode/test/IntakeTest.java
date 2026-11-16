
package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.CRServo;

@TeleOp(name = "Motor and CR Servo Test", group = "Components")
public class IntakeTest extends LinearOpMode {

    private DcMotorEx motor;
    private CRServo servo;

    double motorPower = 0.50;
    static final double POWER_STEP = 0.05;

    boolean motorOn = false;

    // CR Servo direction
    boolean servoForward = true;

    // Button tracking
    boolean lastA = false;
    boolean lastUp = false;
    boolean lastDown = false;
    boolean lastB = false;

    @Override
    public void runOpMode() {

        motor = hardwareMap.get(DcMotorEx.class, "o1");
        servo = hardwareMap.get(CRServo.class, "s1");

        motor.setDirection(DcMotorEx.Direction.FORWARD);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        servo.setDirection(CRServo.Direction.FORWARD);

        telemetry.addLine("Motor and CR Servo Test");
        telemetry.addLine("A = Toggle Motor");
        telemetry.addLine("D-pad Up/Down = Adjust Motor Power");
        telemetry.addLine("B = Toggle CR Servo Direction");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            boolean a = gamepad1.a;
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            boolean b = gamepad1.b;

            // Toggle motor ON/OFF
            if (a && !lastA) {
                motorOn = !motorOn;
            }

            // Adjust motor power
            if (up && !lastUp) {
                motorPower += POWER_STEP;
            }

            if (down && !lastDown) {
                motorPower -= POWER_STEP;
            }

            // Keep motor power between 0 and 1
            motorPower = Math.max(0.0, Math.min(1.0, motorPower));
            motorPower = Math.round(motorPower * 100.0) / 100.0;

            // Toggle CR Servo direction
            if (b && !lastB) {
                servoForward = !servoForward;
            }

            // Save button states
            lastA = a;
            lastUp = up;
            lastDown = down;
            lastB = b;

            // Apply motor power
            motor.setPower(motorOn ? motorPower : 0);

            // Apply CR Servo power
            servo.setPower(servoForward ? 1.0 : -1.0);

            // Telemetry
            telemetry.addData("Motor", motorOn ? "ON" : "OFF");
            telemetry.addData("Motor Power", "%.0f%%", motorPower * 100);

            telemetry.addData("CR Servo Direction",
                    servoForward ? "FORWARD" : "REVERSE");

            telemetry.update();
        }

        // Stop everything when OpMode ends
        motor.setPower(0);
        servo.setPower(0);
    }
}
