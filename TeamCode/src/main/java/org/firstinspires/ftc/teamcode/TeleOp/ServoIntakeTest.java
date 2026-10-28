package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class ServoIntakeTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private CRServo intake1 = null;
    private CRServo intake2 = null;
//    private AnalogInput f;
//    private AnalogInput f1;
    boolean isServoRunning = false;
    boolean previousA = false;



    boolean lastA = false;

    public void runOpMode() {
        intake1 = hardwareMap.get(CRServo.class, "intake1");
        intake2 = hardwareMap.get(CRServo.class, "intake2");
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
                intake1.setPower(sp1);
                intake2.setPower(sp2);
            } else {
                intake1.setPower(0);
                intake2.setPower(0);
                telemetry.addData("Intake1", intake1.getPower());
                telemetry.addData("Intake2", intake2.getPower());
//            telemetry.addData("f", f.getVoltage());
//            telemetry.addData("f1", f1.getVoltage());
                telemetry.update();
            }
        }
    }
}
