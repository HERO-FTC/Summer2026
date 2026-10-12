package org.firstinspires.ftc.teamcode.mechanisms;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class TestColor {
    NormalizedColorSensor colorSensor;
    public enum DetectedColor{
        RED,
        BLUE,
        YELLOW,
        UNKNOWN,
    }

    public void init (HardwareMap hardwareMap){
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color_distance");
        colorSensor.setGain(5);
    }

    public DetectedColor getDetectedColor(Telemetry telemetry){
        NormalizedRGBA colors = colorSensor.getNormalizedColors(); // return 4 values

        float normRed, normGreen, normBlue;
        normRed = colors.red/colors.alpha;
        normGreen = colors.green/colors.alpha;
        normBlue = colors.blue/colors.alpha;

        telemetry.addData("red", normRed);
        telemetry.addData("green", normGreen);
        telemetry.addData("blue", normBlue);

        //TODO add values in if statements for specific colors added
        /*
        red,green,blue
        RED =
        YELLOW =
        BLUE =
        if (normRed >  && normGreen <  && normBlue < ) {
            return DetectedColor.RED;
        }

        else if (normRed >  && normGreen >  && normBlue <  ){
            return DetectedColor.YELLOW;
        }

        else if (normRed <  && normGreen <  && normBlue >  ){
            return DetectedColor.BLUE;
        }

        else{
            return DetectedColor.UNKNOWN;
        }
        */


        return DetectedColor.UNKNOWN;

    }

}
