package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.RoboInit;

public class ShooterTurretSubsystemV2 {

    private final DcMotorEx outtake;
    private final Servo brake;
    private final CRServo kicker;

    // private final Servo turret;

    private static final double BRAKE_ON = 0.47;
    private static final double BRAKE_OFF = 0.51;

    public ShooterTurretSubsystemV2(RoboInit robot) {

        outtake = robot.outtake;
        brake = robot.brake;
        kicker = robot.kicker;

        // turret = robot.turret;

        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Automatically spin kicker backward when subsystem initializes
        kicker.setPower(-1.0);
    }

    public void startOuttake() {
        outtake.setPower(74.0);
    }

    public void stopOuttake() {
        outtake.setPower(0);
    }

    public void runOuttake(double power) {
        outtake.setPower(power);
    }

    public void setBrake(boolean engaged) {
        brake.setPosition(engaged ? BRAKE_ON : BRAKE_OFF);
    }

    public void runKicker(double power) {
        kicker.setPower(power);
    }

    public double getOuttakeRPM() {
        return outtake.getVelocity() * 60.0 / 28.0;
    }

    public void stop() {
        outtake.setPower(0);
        kicker.setPower(0);
    }
}