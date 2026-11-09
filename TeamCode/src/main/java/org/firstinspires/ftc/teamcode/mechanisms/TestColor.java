package org.firstinspires.ftc.teamcode.mechanisms;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class TestColor {
    NormalizedColorSensor colorSensor;
    DistanceSensor distanceSensor;

    //double maxDistance = ;
    //double minDistance = ;
    public enum DetectedColor{
        RED,
        BLUE,
        YELLOW,
        UNKNOWN
    }

    public void init (HardwareMap hardwareMap){
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color_distance");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor_color_distance");
        colorSensor.setGain(8);
    }


    public DetectedColor getDetectedColor(Telemetry telemetry){
        NormalizedRGBA colors = colorSensor.getNormalizedColors(); // return 4 values
        double distanceCm = distanceSensor.getDistance(DistanceUnit.CM);
        float normRed, normGreen, normBlue;

        normRed = colors.red/colors.alpha;
        normGreen = colors.green/colors.alpha;
        normBlue = colors.blue/colors.alpha;

        telemetry.addData("distance", distanceCm);
        telemetry.addData("red", normRed);
        telemetry.addData("green", normGreen);
        telemetry.addData("blue", normBlue);

        //TODO distance with no ball in front of center, distance  when ball is sitting in front of intake, distance where color starts to flicker
        /*
        if (distanceCm > maxDistance){
        return DetectedColor.UNKNOWN;
        }



        */

        if (normRed > 0.5 && normGreen < 0.5 && normBlue < 0.4) {
            return DetectedColor.RED;
        }

        else if (normRed > 0.4 && normGreen > 0.6 && normBlue < 0.3 ){
            return DetectedColor.YELLOW;
        }

        else if (normRed < 0.3 && normGreen < 0.5 && normBlue > 0.5 ){
            return DetectedColor.BLUE;
        }

        else{
            return DetectedColor.UNKNOWN;
        }


    }

}
