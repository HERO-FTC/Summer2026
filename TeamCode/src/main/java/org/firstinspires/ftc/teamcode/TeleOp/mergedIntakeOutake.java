package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "mergedIntakeOutake", group = "Components")

public class mergedIntakeOutake extends LinearOpMode {
    double inTargetVelocity = 1500.0; // ticks per second
    double outTargetVelocity = 2800.0; // ticks per second
    private DcMotorEx in = null;
    private DcMotorEx out1 = null;
    private DcMotorEx out2 = null;
    boolean inOn = false;
    boolean outOn = false;
    boolean lastA = false;
    boolean lastB = false;

    @Override
    public void runOpMode() {
        in = hardwareMap.get(DcMotorEx.class, "in");
        in.setDirection(DcMotorEx.Direction.FORWARD);
        in.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        in.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        out1 = hardwareMap.get(DcMotorEx.class, "o1");
        out2 = hardwareMap.get(DcMotorEx.class, "o2");

        out1.setDirection(DcMotorEx.Direction.FORWARD);
        out2.setDirection(DcMotorEx.Direction.REVERSE);

        out1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        out2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        out1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        out2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean a = gamepad1.x;
            boolean b = gamepad1.y;
            // Flip only on the moment the button goes from up to down
            if (a && !lastA) {
                inOn = !inOn;
            }
            lastA = a;
            if (inOn) {
                in.setVelocity(inTargetVelocity);
            } else {
                in.setVelocity(0);
            }
            if (b && !lastB) {
                outOn = !outOn;
            }
            lastB = b;

            if (outOn) {
                out1.setVelocity(outTargetVelocity);
                out2.setVelocity(outTargetVelocity);
            } else {
                out1.setVelocity(0);
                out2.setVelocity(0);
            }
            telemetry.addData("Outtake", inOn ? "ON" : "OFF");
            telemetry.addData("Target", inTargetVelocity);
            telemetry.addData("o1 vel", in.getVelocity());
            telemetry.update();

            telemetry.addData("Outtake", outOn ? "ON" : "OFF");
            telemetry.addData("Target", outTargetVelocity);
            telemetry.addData("o1 vel", out1.getVelocity());
            telemetry.addData("o2 vel", out2.getVelocity());
            telemetry.update();
        }
    }
}