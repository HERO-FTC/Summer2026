
package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Single Motor Test", group = "Components")
public class OutakeTest extends LinearOpMode {

    private DcMotorEx motor;

    double motorPower = 0.50;
    static final double POWER_STEP = 0.05;

    boolean motorOn = false;

    boolean lastA = false;
    boolean lastUp = false;
    boolean lastDown = false;

    @Override
    public void runOpMode() {

        motor = hardwareMap.get(DcMotorEx.class, "o1");

        motor.setDirection(DcMotorEx.Direction.FORWARD);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Single Motor Test");
        telemetry.addLine("A = Toggle motor");
        telemetry.addLine("D-pad Up/Down = Adjust power");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            boolean a = gamepad1.a;
            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;

            // Toggle motor on/off
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

            // Keep power between 0 and 1
            motorPower = Math.max(0.0, Math.min(1.0, motorPower));
            motorPower = Math.round(motorPower * 100.0) / 100.0;

            // Save button states
            lastA = a;
            lastUp = up;
            lastDown = down;

            // Apply power
            motor.setPower(motorOn ? motorPower : 0);

            // Telemetry
            telemetry.addData("Motor", motorOn ? "ON" : "OFF");
            telemetry.addData("Power", "%.0f%%", motorPower * 100);
            telemetry.update();
        }

        // Stop motor when OpMode ends
        motor.setPower(0);
    }
}
