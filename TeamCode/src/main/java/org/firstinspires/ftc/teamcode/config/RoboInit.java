
package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class RoboInit {

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    public DcMotorEx intake;
    public DcMotorEx outtake;

    public Servo brake;
    public CRServo kicker;
    public Servo turret;
    public Servo flowerBar;
    public Servo gate;

    public RoboInit(HardwareMap hardwareMap) {

        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        outtake = hardwareMap.get(DcMotorEx.class, "outtake");

        brake = hardwareMap.get(Servo.class, "brake");
       // kicker = hardwareMap.get(CRServo.class, "kick");
        turret = hardwareMap.get(Servo.class, "turretAim");
        gate = hardwareMap.get(Servo.class, "gate");
        flowerBar = hardwareMap.get(Servo.class, "flower");
    }
}