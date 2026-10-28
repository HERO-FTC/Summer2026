package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "mergedIntakeOutake", group = "Components")
public class mergedIntakeOutake extends LinearOpMode {

    // starting speeds (ticks per second), the bumpers change these live
    double inTargetVelocity = 1500.0;
    double outTargetVelocity = 2800.0;


    // how much each bumper press changes the selected speed, and the limits
    static final double VELOCITY_STEP = 50.0;
    static final double MIN_VELOCITY = 0.0;
    static final double MAX_VELOCITY = 2800.0;   // raise this if your motors can go faster

    private DcMotorEx in = null;
    private DcMotorEx out1 = null;
    private DcMotorEx out2= null;

    boolean running = false;          // Y: everything on/off
    boolean adjustingIntake = false;  // A: false = bumpers change outtake, true = intake

    // last frame's button states, so we only react the moment a button is pressed
    boolean lastY = false;
    boolean lastA = false;
    boolean lastLB = false;
    boolean lastRB = false;

    @Override
    public void runOpMode() {
        in = hardwareMap.get(DcMotorEx.class, "in");
        in.setDirection(DcMotorEx.Direction.FORWARD);
        in.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        in.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        out1 = hardwareMap.get(DcMotorEx.class, "o1");
        out1.setDirection(DcMotorEx.Direction.FORWARD);
        out1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        out1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        out2 = hardwareMap.get(DcMotorEx.class, "o2");
        out2.setDirection(DcMotorEx.Direction.FORWARD);
        out2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        out2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean y = gamepad1.y;
            boolean a = gamepad1.a;
            boolean lb = gamepad1.left_bumper;
            boolean rb = gamepad1.right_bumper;

            // Y: flip on/off only on the moment the button goes down
            if (y && !lastY) running = !running;

            // A: choose which speed the bumpers will change
            if (a && !lastA) adjustingIntake = !adjustingIntake;

            // bumpers: raise / lower the selected speed
            double change = 0;
            if (rb && !lastRB) change += VELOCITY_STEP;
            if (lb && !lastLB) change -= VELOCITY_STEP;

            if (change != 0) {
                if (adjustingIntake) {
                    inTargetVelocity = clamp(inTargetVelocity + change);
                } else {
                    outTargetVelocity = clamp(outTargetVelocity + change);
                    outTargetVelocity = clamp(outTargetVelocity + change);
                }
            }

            // remember this frame's buttons for next time
            lastY = y;
            lastA = a;
            lastLB = lb;
            lastRB = rb;

            // drive the motors
            in.setVelocity(running ? inTargetVelocity : 0);
            out1.setVelocity(running ? outTargetVelocity : 0);

            telemetry.addData("Running", running ? "ON" : "OFF");
            telemetry.addData("Adjusting", adjustingIntake ? "INTAKE" : "OUTTAKE");
            telemetry.addData("Intake target", inTargetVelocity);
            telemetry.addData("Intake vel", in.getVelocity());
            telemetry.addData("Outtake target", outTargetVelocity);
            telemetry.addData("Outtake vel", out1.getVelocity());
            telemetry.update();
        }

        // make sure everything stops when the OpMode ends
        in.setVelocity(0);
        out1.setVelocity(0);
    }

    // keeps a speed between the min and max limits
    private double clamp(double v) {
        return Math.max(MIN_VELOCITY, Math.min(MAX_VELOCITY, v));
    }
}