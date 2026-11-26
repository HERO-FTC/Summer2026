
package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Stopper")
public class Stopper extends LinearOpMode {

    @Override
    public void runOpMode() {

        Servo stopper = hardwareMap.get(Servo.class, "stopper");

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                stopper.setPosition(0.75);
            }

            if (gamepad1.b) {
                stopper.setPosition(0.25);
            }
        }
    }
}
