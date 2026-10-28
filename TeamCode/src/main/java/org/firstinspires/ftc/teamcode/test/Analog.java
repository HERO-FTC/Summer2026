
package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@TeleOp(name = "laserAnalogExample")
public class Analog extends LinearOpMode {

    private AnalogInput laserAnalog;

    private static final double MAX_VOLTS = 3.3;
    private static final double MAX_DISTANCE_MM = 1000.0;

    // Adjust these after testing your sensor.
    private static final double DETECT_VOLTAGE = 1.0;
    private static final double CLEAR_VOLTAGE = 1.2;

    private int pollenCount = 0;
    private boolean readyToCount = true;

    @Override
    public void runOpMode() {

        laserAnalog = hardwareMap.get(
                AnalogInput.class, "laserAnalogInput"
        );

        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double volts = laserAnalog.getVoltage();

            // Approximate distance conversion.
            double distanceMM =
                    (volts / MAX_VOLTS) * MAX_DISTANCE_MM;

            // Re-arm when the sensor sees a clear path.
            if (volts > CLEAR_VOLTAGE) {
                readyToCount = true;
            }

            // Count once when a new pollen is detected.
            if (volts < DETECT_VOLTAGE && readyToCount) {
                pollenCount++;
                readyToCount = false;
            }

            telemetry.addData("Voltage (V)", "%.3f", volts);
            telemetry.addData("Distance (mm)", "%.1f", distanceMM);
            telemetry.addData("Pollen Count", pollenCount);
            telemetry.update();
        }
    }
}
