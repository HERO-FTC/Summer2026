package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.TestColor;

@TeleOp
public class colorSensor {
    TestColor color = new TestColor();
    public void init(){
        color.init(hardwareMap);
}

    public void loop(){

    }

}
