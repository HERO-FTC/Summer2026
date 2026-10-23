package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotInit {

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    public DcMotorEx intakeOuter;
    public DcMotorEx intakeInner;

    public DcMotorEx flywheel;

    public Servo flowerServo;
    public CRServo turret;

    public RobotInit(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        intakeOuter = hardwareMap.get(DcMotorEx.class, "intakeOuter");
        intakeInner = hardwareMap.get(DcMotorEx.class, "intakeInner");

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        flowerServo = hardwareMap.get(Servo.class, "flowerServo");
        turret = hardwareMap.get(CRServo.class, "turret");
    }
}