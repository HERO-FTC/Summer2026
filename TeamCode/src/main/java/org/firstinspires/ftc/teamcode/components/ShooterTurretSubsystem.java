
package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.config.RobotInit;

public class ShooterTurretSubsystem {

    private final DcMotorEx flywheel;
    private final Servo flowerServo;
    private final CRServo turret;

    private static final double FLOWER_BAR_UP = 0.57;
    private static final double FLOWER_BAR_DOWN = 0.23;
    private static final double TARGET_RPM = 6000*0.65;

    private final double ticksPerRev;

    public ShooterTurretSubsystem(RobotInit config) {
        flywheel = config.flywheel;
        flowerServo = config.flowerServo;
        turret = config.turret;

        ticksPerRev = 28;

        flywheel.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.FLOAT
        );

        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flowerServo.setPosition(FLOWER_BAR_UP);
        turret.setPower(0);
    }

    public void startFlywheel() {
        flywheel.setVelocity(TARGET_RPM * ticksPerRev / 60.0);
    }

    public void setFlywheelRPM(double rpm) {
        flywheel.setVelocity(rpm * ticksPerRev / 60.0);
    }

    public void stopFlywheel() {
        flywheel.setVelocity(0);
    }

    public double getFlywheelRPM() {
        return flywheel.getVelocity() * 60.0 / ticksPerRev;
    }

    public void flowerBarUp() {
        flowerServo.setPosition(FLOWER_BAR_UP);
    }

    public void flowerBarDown() {
        flowerServo.setPosition(FLOWER_BAR_DOWN);
    }

    public void rotateTurret(double power) {
        turret.setPower(Math.max(-1.0, Math.min(1.0, power)));
    }

    public void stopTurret() {
        turret.setPower(0);
    }

    public void stop() {
        stopFlywheel();
        stopTurret();
    }
}
