package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.intakeSubsystem;
import org.firstinspires.ftc.teamcode.mechanisms.TestColor;
import org.firstinspires.ftc.teamcode.mechanisms.TestColor.DetectedColor;
import org.firstinspires.ftc.teamcode.config.RobotInit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@TeleOp
public class colorSensor extends OpMode {
    RobotInit robotConfig;
    private intakeSubsystem intake;
    TestColor color = new TestColor();

    DetectedColor detectedColor;
    DetectedColor distanceCm;

    public void init(){
        robotConfig = new RobotInit(hardwareMap);
        intake = new intakeSubsystem(robotConfig);
        color.init(hardwareMap);
}

    public void loop(){
        detectedColor = color.getDetectedColor(telemetry);
        telemetry.addData("Color Detected", detectedColor);
        telemetry.addData("Distance", distanceCm);
        //This one is for when we're on red
//        if (minDistance < distanceCm < maxDistance && detectedColor == DetectedColor.BLUE){
//            intake.stop();
//            telemetry.addLine("BLUE BAD");
//        }

        //This one is for when we're on blue
//        if (minDistance < distanceCm < maxDistance && detectedColor == DetectedColor.RED){
//            intake.stop();
//            telemetry.addLine("RED BAD");
//        }
    }
}

