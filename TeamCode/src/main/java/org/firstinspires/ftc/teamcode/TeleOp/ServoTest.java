package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class ServoTest extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();

    private CRServo test = null;
    boolean isServoRunning = false;
    boolean previousA = false;


    public void runOpMode() {
        test = hardwareMap.get(CRServo.class, "test");
//        f = hardwareMap.get(AnalogInput.class, "f");
//        f1 = hardwareMap.get(AnalogInput.class, "f1");

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            boolean a = gamepad1.a;

            double sp1 = -0.99;
            double sp2 = 0.99;
            boolean currentA = gamepad1.a;

            if (currentA && !previousA) {
                isServoRunning = !isServoRunning;
            }
            previousA = currentA;
            if (isServoRunning) {
                test.setPower(sp1);
            } else {
                test.setPower(0);
                telemetry.addData("Intake1", test.getPower());
                telemetry.update();
            }

        }
    }
}
