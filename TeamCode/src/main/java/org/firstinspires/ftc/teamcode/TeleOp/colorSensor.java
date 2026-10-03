package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.LED;

@TeleOp
public class colorSensor extends LinearOpMode {
    ColorSensor cs1;
//    ColorSensor cs2;
    private LED redLed;
    private LED greenLed;

    @Override
    public void runOpMode() {
        cs1 = hardwareMap.get(ColorSensor.class, "Color");
        redLed = hardwareMap.get(LED.class, "led_red");
        greenLed = hardwareMap.get(LED.class, "led_green");

        waitForStart();

        while (opModeIsActive()) {
            Telemetry();
        }
    }

    private void Telemetry(){
    telemetry.clear();
        telemetry.addData("Red", cs1.red());
        telemetry.addData("Green", cs1.green());
        telemetry.addData("Blue", cs1.blue());
        telemetry.update();
    }
}