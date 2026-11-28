
package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.RoboInit;

public class ShooterTurretSubsystemV2 {

    private final DcMotorEx outtake;
    private final Servo brake;
    private final Servo flowerBar;
    private final Servo stopper;

    private static final double BRAKE_ON = 0.001;
    private static final double BRAKE_OFF = 0.51;

    private static final double FLOWER_SERVO_DOWN = 0.6;
    private static final double FLOWER_SERVO_UP = 0.6;

    private static final double STOPPER_CLOSED = 0.2;
    private static final double STOPPER_OPEN = 0.61;

    private static final double TARGET_RPM = 6000 * 0.50;

    public ShooterTurretSubsystemV2(RoboInit robot) {

        outtake = robot.outtake;
        brake = robot.brake;
        stopper = robot.gate;
        flowerBar = robot.flowerBar;

        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        stopper.setPosition(STOPPER_CLOSED);

        // Flower bar initialization disabled.
        // flowerBar.setPosition(FLOWER_SERVO_UP);
    }

    public void flowerServoUp() {
        // Disabled temporarily.
        // flowerBar.setPosition(FLOWER_SERVO_UP);
    }

    public void flowerServoDown() {
        // Disabled temporarily.
        // flowerBar.setPosition(FLOWER_SERVO_DOWN);
    }

    public void startFlywheel() {
        outtake.setVelocity(TARGET_RPM * 28 / 60.0);
    }

    public void stopFlywheel() {
        outtake.setVelocity(0);
    }

    public void runOuttake(double power) {
        outtake.setPower(power);
    }

    public void setBrake(boolean engaged) {
        brake.setPosition(engaged ? BRAKE_ON : BRAKE_OFF);
    }

    public void runKicker(double power) {
    }

    public void setStopper(boolean open) {
        stopper.setPosition(open ? STOPPER_OPEN : STOPPER_CLOSED);
    }

    public double getOuttakeRPM() {
        return outtake.getVelocity() * 60.0 / 28.0;
    }

    public void stop() {
        outtake.setPower(0);
        stopper.setPosition(STOPPER_CLOSED);

        // Flower bar shutdown command disabled.
        // flowerBar.setPosition(FLOWER_SERVO_UP);
    }
}
