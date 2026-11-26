
package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Test", group = "Testing")
public class GobildaServoTest extends LinearOpMode {

    private Servo stopper;

    private static final double STOPPER_UP = 0.51;
    private static final double STOPPER_DOWN = 0.30;

    @Override
    public void runOpMode() {

        // INITIALIZE STOPPER
        stopper = hardwareMap.get(Servo.class, "stopper");

        // SET INITIAL POSITION
        stopper.setPosition(STOPPER_UP);

        telemetry.addLine("STOPPER SERVO TEST");
        telemetry.addLine("--------------------");
        telemetry.addLine("A = STOPPER DOWN");
        telemetry.addLine("B = STOPPER UP");
        telemetry.addLine("X = POSITION 0.00");
        telemetry.addLine("Y = POSITION 1.00");
        telemetry.addData("Current Position", STOPPER_UP);

        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {

            // A = DOWN
            if (gamepad1.a) {
                stopper.setPosition(STOPPER_DOWN);
            }

            // B = UP
            else if (gamepad1.b) {
                stopper.setPosition(STOPPER_UP);
            }

            // X = POSITION 0.00
            else if (gamepad1.x) {
                stopper.setPosition(0.00);
            }

            // Y = POSITION 1.00
            else if (gamepad1.y) {
                stopper.setPosition(1.00);
            }

            telemetry.addLine("STOPPER SERVO TEST");

            telemetry.addData(
                    "Command",
                    gamepad1.a ? "DOWN" :
                            gamepad1.b ? "UP" :
                            gamepad1.x ? "0.00" :
                            gamepad1.y ? "1.00" : "HOLDING"
            );

            telemetry.update();
        }
    }
}
