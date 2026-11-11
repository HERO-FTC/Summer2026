package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
public class robotHardwareT {

    public static final String TURRET_SERVO_NAME = "turret";
    public static final String SHOOTER_MOTOR_NAME = "shooter";
    public static final String WEBCAM_NAME = "Webcam 1";

    public Servo turretServo;
    public DcMotorEx shooterMotor;
    public WebcamName webcam;

    // Initialize robot hardware
    public void init(HardwareMap hardwareMap) {

        // Connect turret servo
        turretServo = hardwareMap.get(Servo.class, TURRET_SERVO_NAME);

        // Connect shooter motor
        shooterMotor = hardwareMap.get(DcMotorEx.class, SHOOTER_MOTOR_NAME);

        // Connect webcam
        webcam = hardwareMap.get(WebcamName.class, WEBCAM_NAME);

        // Set safe initial shooter settings
        shooterMotor.setPower(0);
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void stopShooter() {
        if (shooterMotor != null) {
            shooterMotor.setPower(0);
        }
    }
}



