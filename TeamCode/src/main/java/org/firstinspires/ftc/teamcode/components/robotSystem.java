
package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class robotSystem {

    // Motor target speeds in RPM
    public static final double OUT1_RPM = 6000.0;
    public static final double OUT2_RPM = 435.0;
    public static final double INTAKE_RPM = 1620.0;

    private final DcMotorEx out1;
    private final DcMotorEx out2;
    private final DcMotorEx intake;

    private double speedMultiplier = 1.0;

    private boolean outtakeOn = false;
    private boolean intakeOn = false;

    public robotSystem(HardwareMap hardwareMap) {

        // Initialize motors using configured hardware names
        out1 = hardwareMap.get(DcMotorEx.class, "o1");
        out2 = hardwareMap.get(DcMotorEx.class, "o2");
        intake = hardwareMap.get(DcMotorEx.class, "in");

        // Motor directions from your original code
        out1.setDirection(DcMotor.Direction.REVERSE);
        out2.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.REVERSE);

        // Enable encoder-based velocity control
        out1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        out2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Start with all motors stopped
        stopAll();
    }

    // Convert RPM to encoder ticks per second
    private double rpmToTicksPerSecond(
            DcMotorEx motor, double rpm) {

        double ticksPerRevolution =
                motor.getMotorType().getTicksPerRev();

        return rpm * ticksPerRevolution / 60.0;
    }

    // Set an individual motor's target RPM
    public void setOut1RPM(double rpm) {
        out1.setVelocity(
                rpmToTicksPerSecond(out1, rpm)
                        * speedMultiplier
        );
    }

    public void setOut2RPM(double rpm) {
        out2.setVelocity(rpmToTicksPerSecond(out2, rpm) * speedMultiplier);
    }

    public void setIntakeRPM(double rpm) {
        intake.setVelocity(rpmToTicksPerSecond(intake, rpm) * speedMultiplier);
    }

    // Run both outtake motors at their configured speeds
    public void startOuttake() {
        outtakeOn = true;

        setOut1RPM(OUT1_RPM);
        setOut2RPM(OUT2_RPM);
    }

    // Stop both outtake motors
    public void stopOuttake() {
        outtakeOn = false;

        out1.setVelocity(0);
        out2.setVelocity(0);
        out1.setPower(0);
        out2.setPower(0);
    }

    // Run the intake at its configured speed
    public void startIntake() {
        intakeOn = true;
        setIntakeRPM(INTAKE_RPM);
    }

    // Stop the intake
    public void stopIntake() {
        intakeOn = false;

        intake.setVelocity(0);
        intake.setPower(0);
    }

    // Run all three motors
    public void startAll() {
        startOuttake();
        startIntake();
    }

    // Stop all three motors
    public void stopAll() {
        outtakeOn = false;
        intakeOn = false;

        out1.setVelocity(0);
        out2.setVelocity(0);
        intake.setVelocity(0);

        out1.setPower(0);
        out2.setPower(0);
        intake.setPower(0);
    }

    // Toggle the complete system
    public void toggleAll() {
        if (outtakeOn || intakeOn) {
            stopAll();
        } else {
            startAll();
        }
    }

    // Adjust all target speeds proportionally, from 0 to 100%
    public void setSpeedMultiplier(double multiplier) {
        speedMultiplier = Math.max(
                0.0, Math.min(1.0, multiplier)
        );

        // Immediately update any motors that are running
        if (outtakeOn) {
            setOut1RPM(OUT1_RPM);
            setOut2RPM(OUT2_RPM);
        }

        if (intakeOn) {
            setIntakeRPM(INTAKE_RPM);
        }
    }

    public double getSpeedMultiplier() {
        return speedMultiplier;
    }

    public boolean isOuttakeOn() {
        return outtakeOn;
    }

    public boolean isIntakeOn() {
        return intakeOn;
    }

    // Read measured encoder velocity in RPM
    public double getOut1RPM() {
        return out1.getVelocity()
                * 60.0 / out1.getMotorType().getTicksPerRev();
    }

    public double getOut2RPM() {
        return out2.getVelocity()
                * 60.0 / out2.getMotorType().getTicksPerRev();
    }

    public double getIntakeRPM() {
        return intake.getVelocity()
                * 60.0 / intake.getMotorType().getTicksPerRev();
    }
}
