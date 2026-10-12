package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;




@TeleOp(name="chassisDrive", group="Linear OpMode")
@Disabled
public class chassisDrive extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // --- Endgame timer configuration ---
    static final double TELEOP_DURATION_S = 120.0;   // official TeleOp length
    static final double SHUTDOWN_BEFORE_S  = 0.1;    // shut down 2 seconds early (T = 118s)
    private boolean endgameShutdownApplied = false;  // ensure we only hard-stop once

    // Wheels

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private double leftFrontPower;
    private double rightFrontPower;
    private double leftBackPower;
    private double rightBackPower;

    // Intake

    double inTargetVelocity = 1500.0; // ticks per second
    private DcMotorEx in = null;
    boolean inOn = false;
    boolean lastA = false;

    // Outtake

    double outTargetVelocity = 2800.0; // ticks per second
    private DcMotorEx out1 = null;
    boolean outOn = false;
    boolean lastB = false;

    @Override
    public void runOpMode() {
        initHardware();
        waitForStart();
        runtime.reset();
        while (opModeIsActive()) {
            handleDriving();
            // Telemetry
            Telemetry();
        }
    }

    private void initHardware() {

        // Wheels
        leftFrontDrive = hardwareMap.get(DcMotor.class, "LF");
        leftBackDrive = hardwareMap.get(DcMotor.class, "LB");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RF");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RB");
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Intake
        in = hardwareMap.get(DcMotorEx.class, "in");
        in.setDirection(DcMotorEx.Direction.REVERSE);
        in.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        in.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        in.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        // Outtake
        out1 = hardwareMap.get(DcMotorEx.class, "o1");

        out1.setDirection(DcMotorEx.Direction.REVERSE);

        out1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
//        out1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        out1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

         // Turret

        // servos

        // Color Sensors

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
    private void handleDriving() {
        // Endgame timer computation
        double elapsedTeleOp = runtime.seconds();
        boolean endgameActive = elapsedTeleOp >= (TELEOP_DURATION_S - SHUTDOWN_BEFORE_S);

        if (endgameActive && !endgameShutdownApplied) {
            inOn = false;
            in.setPower(0.0);

//            outTargetVelocity = 0.0;
            out1.setPower(0.0);

    }

        //Wheels
        double max;

        double vertical = gamepad1.left_stick_y;
        double lateral = -gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;
        ElapsedTime armTime = new ElapsedTime();

        leftFrontPower = (vertical + lateral + rotate);
        rightFrontPower = (vertical - lateral - rotate);
        leftBackPower = (vertical - lateral + rotate);
        rightBackPower = (vertical + lateral - rotate);

        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        if ((gamepad1.right_trigger <= 0) && (gamepad1.left_trigger <=0)) {
            leftFrontPower /= 2;
            rightFrontPower /= 2;
            leftBackPower /= 2;
            rightBackPower /= 2;
        }

        else if ((gamepad1.right_trigger > 0) && (gamepad1.left_trigger == 0)) {
            leftFrontPower = (vertical + lateral + rotate);
            rightFrontPower = (vertical - lateral - rotate);
            leftBackPower = (vertical - lateral + rotate);
            rightBackPower = (vertical + lateral - rotate);
        }

        else if (gamepad1.left_trigger > 0 ) {
            leftFrontPower /= 4;
            rightFrontPower /= 4;
            leftBackPower /= 4;
            rightBackPower /= 4;
        }
        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

        //Intake

        boolean a = gamepad1.x;
        if (a && !lastA) {
            inOn = !inOn;
        }
        lastA = a;
        if (inOn) {
//            in.setVelocity(inTargetVelocity);
            in.setPower(0.8);
        } else {
//            in.setVelocity(0);
            in.setPower(0);
        }

        //Outtake

        boolean b = gamepad1.y;
        if (b && !lastB) {
            outOn = !outOn;
        }
        lastB = b;

        if (outOn) {
//            out1.setVelocity(outTargetVelocity);
            out1.setPower(0.8);
        } else {
            out1.setPower(0);
        }

    }

    private void Telemetry() {
        // Clear previous telemetry data (optional, but keeps the display clean)
        telemetry.clear();

        telemetry.addLine("....................................");
        telemetry.addData("Outtake", inOn ? "ON" : "OFF");
//        telemetry.addData("Target", inTargetVelocity);
//        telemetry.addData("o1 vel", in.getVelocity());
        telemetry.update();

        telemetry.addData("Outtake", outOn ? "ON" : "OFF");
//        telemetry.addData("Target", outTargetVelocity);
//        telemetry.addData("o1 vel", out1.getVelocity());

        telemetry.update();
    }
}
