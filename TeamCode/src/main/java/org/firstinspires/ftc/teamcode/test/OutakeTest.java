
package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "outake", group = "Components")
public class OutakeTest extends LinearOpMode {

    private DcMotorEx intake;
    private DcMotorEx out1;
    private DcMotorEx intake2;

    // Each motor has its own power setting
    double out1Power = 0.50;
    double out2Power = 0.50;
    double intakePower = 0.50;

    static final double POWER_STEP = 0.05;

    // Motor on/off states
    boolean out1On = false;
    boolean out2On = false;
    boolean intakeOn = false;

    // Previous button states for one-press detection
    boolean lastA, lastB, lastX;
    boolean lastUp, lastDown, lastRight, lastLeft;
    boolean lastRB, lastLB;

    @Override
    public void runOpMode() {

        out1 = hardwareMap.get(DcMotorEx.class, "o1");
        intake2 = hardwareMap.get(DcMotorEx.class, "in2");
        intake = hardwareMap.get(DcMotorEx.class, "in1");

        out1.setDirection(DcMotorEx.Direction.FORWARD);
        intake2.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setDirection(DcMotorEx.Direction.REVERSE);

        out1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Initialized");
        telemetry.addLine("A/B/X = toggle motors");
        telemetry.addLine("D-pad = adjust outtakes");
        telemetry.addLine("Bumpers = adjust intake");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Read buttons
            boolean a = gamepad1.a;
            boolean b = gamepad1.b;
            boolean x = gamepad1.x;

            boolean up = gamepad1.dpad_up;
            boolean down = gamepad1.dpad_down;
            boolean right = gamepad1.dpad_right;
            boolean left = gamepad1.dpad_left;

            boolean rb = gamepad1.right_bumper;
            boolean lb = gamepad1.left_bumper;

            // Toggle each motor independently
            if (a && !lastA) out1On = !out1On;
            if (b && !lastB) out2On = !out2On;
            if (x && !lastX) intakeOn = !intakeOn;

            // Adjust out1 power
            if (up && !lastUp) out1Power += POWER_STEP;
            if (down && !lastDown) out1Power -= POWER_STEP;

            // Adjust out2 power
            if (right && !lastRight) out2Power += POWER_STEP;
            if (left && !lastLeft) out2Power -= POWER_STEP;

            // Adjust intake power
            if (rb && !lastRB) intakePower += POWER_STEP;
            if (lb && !lastLB) intakePower -= POWER_STEP;

            // Keep power between 0 and 1
            out1Power = clipPower(out1Power);
            out2Power = clipPower(out2Power);
            intakePower = clipPower(intakePower);

            // Remember button states
            lastA = a;
            lastB = b;
            lastX = x;

            lastUp = up;
            lastDown = down;
            lastRight = right;
            lastLeft = left;

            lastRB = rb;
            lastLB = lb;

            // Apply power independently
            out1.setPower(out1On ? out1Power : 0);
            intake2.setPower(out2On ? out2Power : 0);
            intake.setPower(intakeOn ? intakePower : 0);

            // Telemetry
            telemetry.addData("Out1", "%s | %d%%",
                    out1On ? "ON" : "OFF", (int) (out1Power * 100));

            telemetry.addData("Out2", "%s | %d%%",
                    out2On ? "ON" : "OFF", (int) (out2Power * 100));

            telemetry.addData("Intake", "%s | %d%%",
                    intakeOn ? "ON" : "OFF", (int) (intakePower * 100));

            telemetry.update();
        }

        // Stop everything when OpMode ends
        out1.setPower(0);
        intake2.setPower(0);
        intake.setPower(0);
    }

    private double clipPower(double power) {
        power = Math.max(0.0, Math.min(1.0, power));
        return Math.round(power * 100.0) / 100.0;
    }
}
