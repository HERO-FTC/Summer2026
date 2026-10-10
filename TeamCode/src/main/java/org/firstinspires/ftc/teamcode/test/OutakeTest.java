package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "outake", group = "Components")

public class OutakeTest extends LinearOpMode {

    // 95% power, open loop (no encoder feedback)
    static final double OUT_POWER = 0.80;

    private DcMotorEx out1 = null;
    private DcMotorEx out2 = null;

    boolean outOn = false;
    boolean lastA = false;

    @Override
    public void runOpMode() {
        out1 = hardwareMap.get(DcMotorEx.class, "o1");
        out2 = hardwareMap.get(DcMotorEx.class, "o2");

        out1.setDirection(DcMotorEx.Direction.REVERSE);
        out2.setDirection(DcMotorEx.Direction.FORWARD);

        // no encoder, just raw power
        out1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        out2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

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
                out1.setPower(OUT_POWER);
                out2.setPower(OUT_POWER);
            } else {
                out1.setPower(0);
                out2.setPower(0);
            }

            telemetry.addData("Outtake", outOn ? "ON" : "OFF");
            telemetry.addData("Power", outOn ? OUT_POWER : 0.0);
            telemetry.update();
        }
    }
}