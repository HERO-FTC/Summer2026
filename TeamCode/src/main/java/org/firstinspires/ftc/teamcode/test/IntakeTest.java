
package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Kicker + Intake + Outtake Test", group = "Components")
public class IntakeTest extends LinearOpMode {

    private CRServo kicker;
    private DcMotorEx intake;
    private DcMotorEx outtake;

    private boolean kickerOn = true;
    private boolean intakeOn = false;
    private boolean outtakeOn = false;

    private boolean kickerReversed = false;

    private boolean lastA = false;
    private boolean lastX = false;
    private boolean lastY = false;
    private boolean lastB = false;

    @Override
    public void runOpMode() {

        kicker = hardwareMap.get(CRServo.class, "kick");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        outtake = hardwareMap.get(DcMotorEx.class, "outake");

        // Kicker configuration
        kicker.setDirection(CRServo.Direction.FORWARD);

        // Intake configuration
        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Outtake configuration
        outtake.setDirection(DcMotorSimple.Direction.FORWARD);
        outtake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        outtake.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Kicker + Intake + Outtake Test");
        telemetry.addLine("A = Toggle Kicker");
        telemetry.addLine("X = Toggle Intake");
        telemetry.addLine("Y = Reverse Kicker / Shoot");
        telemetry.addLine("B = Toggle Outtake");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            boolean a = gamepad1.a;
            boolean x = gamepad1.x;
            boolean y = gamepad1.y;
            boolean b = gamepad1.b;

            // Toggle kicker
            if (a && !lastA) {
                kickerOn = !kickerOn;
            }

            // Toggle intake
            if (x && !lastX) {
                intakeOn = !intakeOn;
            }

            // Reverse kicker for shooting
            if (y && !lastY) {
                kickerReversed = !kickerReversed;
            }

            // Toggle outtake
            if (b && !lastB) {
                outtakeOn = !outtakeOn;
            }

            // Save button states
            lastA = a;
            lastX = x;
            lastY = y;
            lastB = b;

            // Kicker direction and power
            kicker.setDirection(
                    kickerReversed
                            ? CRServo.Direction.REVERSE
                            : CRServo.Direction.FORWARD
            );

            kicker.setPower(kickerOn ? 1.0 : 0.0);

            // Intake at 100%
            intake.setPower(intakeOn ? 1.0 : 0.0);

            // Outtake at 100%
            outtake.setPower(outtakeOn ? 0.76 : 0.0);

            // Telemetry
            telemetry.addData("Kicker", kickerOn ? "ON" : "OFF");
            telemetry.addData("Kicker Direction",
                    kickerReversed ? "SHOOT (+100%)" : "BACKWARD (-100%)");

            telemetry.addData("Intake", intakeOn ? "ON (100%)" : "OFF");
            telemetry.addData("Outtake", outtakeOn ? "ON (100%)" : "OFF");

            telemetry.update();
        }

        // Stop everything
        kicker.setPower(0);
        intake.setPower(0);
        outtake.setPower(0);
    }
}

