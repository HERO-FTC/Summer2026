//package org.firstinspires.ftc.teamcode.components;
//
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.config.RoboInit;
//
//public class ShooterTurretSubsystemV2 {
//
//    private final DcMotorEx outtake;
//    private final Servo brake;
//    private final CRServo kicker;
////    private final Servo flicker;
//
//    // private final Servo turret;
//
//    private static final double BRAKE_ON = 0.47;
//    private static final double BRAKE_OFF = 0.51;
//    private static final double TARGET_RPM = 6000 * 1;
////    private final double ticksPerRev;
//
//
//    public ShooterTurretSubsystemV2(RoboInit robot) {
//
//        outtake = robot.outtake;
//        brake = robot.brake;
//        kicker = robot.kicker;
////        flicker = robot.f
//
//        // turret = robot.turret;
//
////        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
//        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
////        outtake.setDirection(DcMotorSimple.Direction.REVERSE);
//
//        // Automatically spin kicker backward when subsystem initializes
////        kicker.setPower(-1.0);
//        kicker.setPower(1.0);
//    }
//
////    public void startOuttake() {
////        outtake.setPower(0.95);
////    }
//
////    public void stopOuttake() {
////        outtake.setPower(0);
////    }
//
//    public void startFlywheel() {
//        outtake.setVelocity(TARGET_RPM * 28 / 60.0);
//    }
//    public void stopFlywheel() {
//        outtake.setVelocity(0);
//    }
//
//    public void runOuttake(double power) {
//        outtake.setPower(power);
//    }
//
//    public void setBrake(boolean engaged) {
//        brake.setPosition(engaged ? BRAKE_ON : BRAKE_OFF);
//    }
//
//    public void runKicker(double power) {
//        kicker.setPower(power);
//    }
//
//    public double getOuttakeRPM() {
//        return outtake.getVelocity() * 60.0 / 28.0;
//    }
//
//    public void stop() {
//        outtake.setPower(0);
//        kicker.setPower(0);
//    }
//}
package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.RoboInit;

public class ShooterTurretSubsystemV2 {

    private final DcMotorEx outtake;
    private final Servo brake;
    //private final CRServo kicker;
    private final Servo flowerBar;
    private final Servo stopper;

    // BRAKE SERVO POSITIONS
    private static final double BRAKE_ON = 0.47;
    private static final double BRAKE_OFF = 0.51;
    private static final double FLOWER_SERVO_UP = 0.47;
    private static final double FLOWER_SERVO_DOWN = 0.07;

    // STOPPER SERVO POSITIONS
    // These are initial testing values based on your teammates' tests.
    private static final double STOPPER_CLOSED = 0.47;
    private static final double STOPPER_OPEN = 0.51;

    // FLYWHEEL TARGET
    private static final double TARGET_RPM = 6000 * 1;


    public ShooterTurretSubsystemV2(RoboInit robot) {


        outtake = robot.outtake;
        brake = robot.brake;
      //  kicker = robot.kicker;
        stopper = robot.gate;
        flowerBar = robot.flowerBar;
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // KICKER STARTS STOPPED
       // kicker.setPower(0);

        // STOPPER STARTS CLOSED
        stopper.setPosition(STOPPER_CLOSED);
        flowerBar.setPosition(FLOWER_SERVO_UP);
    }
    public void flowerServoUp() {flowerBar.setPosition(FLOWER_SERVO_UP);
    }

    public void flowerServoDown() {flowerBar.setPosition(FLOWER_SERVO_DOWN);
    }



    // FLYWHEEL CONTROL

    public void startFlywheel() {
        outtake.setVelocity(TARGET_RPM * 28 / 60.0);
    }

    public void stopFlywheel() {
        outtake.setVelocity(0);
    }

    public void runOuttake(double power) {
        outtake.setPower(power);
    }


    // BRAKE CONTROL

    public void setBrake(boolean engaged) {
        brake.setPosition(engaged ? BRAKE_ON : BRAKE_OFF);
    }


    // KICKER CONTROL

    public void runKicker(double power) {
       // kicker.setPower(power);
    }


    // STOPPER CONTROL

    public void setStopper(boolean open) {
        stopper.setPosition(open ? STOPPER_OPEN : STOPPER_CLOSED);
    }


    // RPM TELEMETRY

    public double getOuttakeRPM() {
        return outtake.getVelocity() * 60.0 / 28.0;
    }


    // STOP EVERYTHING

    public void stop() {
        outtake.setPower(0);
     //   kicker.setPower(0);

        // Return stopper to closed position when OpMode ends.
        stopper.setPosition(STOPPER_CLOSED);
    }
}