
package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "BioBuzz Drive + Intake Test", group = "Test")
public class Meet2Test1 extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor intake;

    @Override
    public void runOpMode() {

        // Initialize drivetrain motors.
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Initialize intake motor.
        intake = hardwareMap.get(DcMotor.class, "intakeOuter");

        // Set motor directions.
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set zero power behavior.
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Reset motor encoders.
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Set motors to run using encoders.
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        boolean intakeOn = false;

        boolean previousA = false;
        boolean previousUp = false;
        boolean previousDown = false;

        double intakePower = 0.50;

        telemetry.addLine("BioBuzz Drive + Intake Test");
        telemetry.addLine("Gamepad 1 Controls:");
        telemetry.addLine("Left Stick: Drive");
        telemetry.addLine("Right Stick X: Rotate");
        telemetry.addLine("Left Bumper: Slow Mode");
        telemetry.addLine("A: Toggle Intake");
        telemetry.addLine("D-pad Up/Down: Adjust Intake Power");
        telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive()) {

                // DRIVETRAIN
                double forward = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double turn = gamepad1.right_stick_x;

                double driveScale = gamepad1.left_bumper ? 0.4 : 1.0;

                double frontLeftPower = forward + strafe + turn;
                double frontRightPower = forward - strafe - turn;
                double backLeftPower = forward - strafe + turn;
                double backRightPower = forward + strafe - turn;

                double maxPower = Math.max(1.0,
                        Math.max(Math.abs(frontLeftPower),
                                Math.max(Math.abs(frontRightPower),
                                        Math.max(Math.abs(backLeftPower),
                                                Math.abs(backRightPower)))));

                frontLeft.setPower(frontLeftPower / maxPower * driveScale);
                frontRight.setPower(frontRightPower / maxPower * driveScale);
                backLeft.setPower(backLeftPower / maxPower * driveScale);
                backRight.setPower(backRightPower / maxPower * driveScale);

                // INTAKE CONTROLS
                boolean currentA = gamepad1.a;
                boolean currentUp = gamepad1.dpad_up;
                boolean currentDown = gamepad1.dpad_down;

                if (currentA && !previousA) {
                    intakeOn = !intakeOn;
                }

                if (currentUp && !previousUp) {
                    intakePower = Math.min(1.0, intakePower + 0.05);
                }

                if (currentDown && !previousDown) {
                    intakePower = Math.max(0.0, intakePower - 0.05);
                }

                intake.setPower(intakeOn ? intakePower : 0.0);

                previousA = currentA;
                previousUp = currentUp;
                previousDown = currentDown;

                // TELEMETRY
                telemetry.addLine("=== DRIVETRAIN ===");
                telemetry.addData("Forward", "%.2f", forward);
                telemetry.addData("Strafe", "%.2f", strafe);
                telemetry.addData("Turn", "%.2f", turn);
                telemetry.addData("Slow Mode", gamepad1.left_bumper);

                telemetry.addLine("=== INTAKE ===");
                telemetry.addData("Status", intakeOn ? "ON" : "OFF");
                telemetry.addData("Selected Power", "%.2f", intakePower);
                telemetry.addData("Applied Power",
                        "%.2f", intakeOn ? intakePower : 0.0);

                telemetry.update();
            }
        } finally {

            // Stop all five motors.
            frontLeft.setPower(0);
            frontRight.setPower(0);
            backLeft.setPower(0);
            backRight.setPower(0);
            intake.setPower(0);
        }
    }
}
