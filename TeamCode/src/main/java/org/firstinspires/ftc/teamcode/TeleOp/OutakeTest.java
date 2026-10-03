package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@TeleOp(name = "outake", group = "Components")

public class OutakeTest extends LinearOpMode {

    double outTargetVelocity = 2800.0;
    private DcMotorEx out1 = null;
    private DcMotorEx out2 = null;

    boolean outOn = false;
    boolean lastA = false;

    @Override
    public void runOpMode() {
        out1 = hardwareMap.get(DcMotorEx.class, "o1");
        out2 = hardwareMap.get(DcMotorEx.class, "o2");

        out1.setDirection(DcMotorEx.Direction.FORWARD);
        out2.setDirection(DcMotorEx.Direction.REVERSE);

        out1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        out2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        out1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        out2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Optional tuning (defaults are roughly P=10, I=3, D=0, F=0):
        // out1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,
        //         new PIDFCoefficients(10, 3, 0, 0));
        // (needs: import com.qualcomm.robotcore.hardware.PIDFCoefficients;)

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            boolean a = gamepad1.a;

            // Flip only on the moment the button goes from up to down
            if (a && !lastA) {
                outOn = !outOn;
            }
            lastA = a;

            if (outOn) {
                out1.setVelocity(outTargetVelocity);
                out2.setVelocity(outTargetVelocity);
            } else {
                out1.setVelocity(0);
                out2.setVelocity(0);
            }

            telemetry.addData("Outtake", outOn ? "ON" : "OFF");
            telemetry.addData("Target", outTargetVelocity);
            telemetry.addData("o1 vel", out1.getVelocity());
            telemetry.addData("o2 vel", out2.getVelocity());
            telemetry.update();
        }
    }
}