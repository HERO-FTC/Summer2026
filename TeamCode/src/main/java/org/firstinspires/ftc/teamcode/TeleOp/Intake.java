package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Intake", group = "Components")

public class Intake extends LinearOpMode {
    double outTargetVelocity = 1500.0; // ticks per second
    private DcMotorEx in = null;
    boolean inOn = false;
    boolean lastA = false;

    @Override
    public void runOpMode() {
        in = hardwareMap.get(DcMotorEx.class, "in");
        in.setDirection(DcMotorEx.Direction.FORWARD);
        in.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        in.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean a = gamepad1.x;
            // Flip only on the moment the button goes from up to down
            if (a && !lastA) {
                inOn = !inOn;
            }
            lastA = a;
            if (inOn) {
                in.setVelocity(outTargetVelocity);
            } else {
                in.setVelocity(0);
            }
            telemetry.addData("Outtake", inOn ? "ON" : "OFF");
            telemetry.addData("Target", outTargetVelocity);
            telemetry.addData("o1 vel", in.getVelocity());

            telemetry.update();
        }
    }
}