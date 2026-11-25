
package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import java.util.Iterator;

@TeleOp(name = "ServoVoltage Test", group = "Testing")
public class ServoVoltageTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        // Get the Control Hub's voltage sensor
        Iterator<VoltageSensor> sensors =
                hardwareMap.voltageSensor.iterator();

        if (!sensors.hasNext()) {
            telemetry.addLine("No voltage sensor found!");
            telemetry.update();
            return;
        }

        VoltageSensor voltageSensor = sensors.next();

        telemetry.addLine("Voltage Monitor Ready");
        telemetry.addLine("Press PLAY to begin");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Read the current robot battery voltage
            double voltage = voltageSensor.getVoltage();

            // Display the reading
            telemetry.addData("Robot Battery Voltage", "%.2f V", voltage);

            telemetry.update();

            sleep(100);
        }
    }
}
