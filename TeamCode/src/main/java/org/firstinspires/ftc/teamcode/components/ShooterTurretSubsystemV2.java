
package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.config.RoboInit;

public class ShooterTurretSubsystemV2 {

    private final DcMotorEx outtake;
    private final Servo brake;
    private final CRServo kicker;

    private static final double BRAKE_ON = 0.47;
    private static final double BRAKE_OFF = 0.51;

    private static final double TARGET_RPM = 6000.0 * 0.35;

    private final double ticksPerRev;

    public ShooterTurretSubsystemV2(RoboInit robot) {

        outtake = robot.outtake;
        brake = robot.brake;
        kicker = robot.kicker;

        ticksPerRev = outtake.getMotorType().getTicksPerRev();

        outtake.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        outtake.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );

        // PIDF configuration (add coefficients later)
        // outtake.setPIDFCoefficients(
        //         DcMotor.RunMode.RUN_USING_ENCODER,
        //         new PIDFCoefficients(P, I, D, F)
        // );

        // Kicker starts OFF
        kicker.setPower(0);
    }

    public void startOuttake() {
        setOuttakeRPM(TARGET_RPM);
    }

    public void setOuttakeRPM(double rpm) {

        outtake.setVelocity(
                rpm * ticksPerRev / 60.0
        );
    }

    public void stopOuttake() {
        outtake.setVelocity(0);
    }

    public void runOuttake(double power) {

        power = Math.max(-1.0, Math.min(1.0, power));

        setOuttakeRPM(power * TARGET_RPM);
    }

    public void setBrake(boolean engaged) {
        brake.setPosition(
                engaged ? BRAKE_ON : BRAKE_OFF
        );
    }

    public void runKicker(double power) {
        kicker.setPower(
                Math.max(-1.0, Math.min(1.0, power))
        );
    }

    public void stopKicker() {
        kicker.setPower(0);
    }

    public double getOuttakeRPM() {
        return outtake.getVelocity() * 60.0 / ticksPerRev;
    }

    public void stop() {
        stopOuttake();
        stopKicker();
    }
}