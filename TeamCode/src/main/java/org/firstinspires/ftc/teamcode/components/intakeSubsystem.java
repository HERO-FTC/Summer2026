
package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.config.RobotInit;

public class intakeSubsystem {

    private final DcMotorEx outerIntake;
    private final DcMotorEx middleIntake;
    private final DcMotorEx innerIntake;

    private static final double OUTER_RPM = 1150.0 * 0.80;
    private static final double MIDDLE_RPM = OUTER_RPM;
    private static final double INNER_RPM = 1620.0 * 0.99;

    private final double outerTicksPerRev;
    private final double middleTicksPerRev;
    private final double innerTicksPerRev;

    public intakeSubsystem(RobotInit config) {
        outerIntake = config.intakeOuter;
        middleIntake = config.middleIntake;
        innerIntake = config.intakeInner;

        outerTicksPerRev =
                outerIntake.getMotorType().getTicksPerRev();

        middleTicksPerRev =
                middleIntake.getMotorType().getTicksPerRev();

        innerTicksPerRev =
                innerIntake.getMotorType().getTicksPerRev();

        outerIntake.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);
        middleIntake.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);
        innerIntake.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        outerIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        middleIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        innerIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set directions as needed for your robot's mechanism
        outerIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        middleIntake.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void runOuter(double power) {
        outerIntake.setVelocity(
                power * OUTER_RPM * outerTicksPerRev / 60.0);
    }

    public void runMiddle(double power) {
        middleIntake.setVelocity(
                power * MIDDLE_RPM * middleTicksPerRev / 60.0);
    }

    public void runInner(double power) {
        innerIntake.setVelocity(
                power * INNER_RPM * innerTicksPerRev / 60.0);
    }

    // Outer and middle intakes run together
    public void runOuterAndMiddle(double power) {
        runOuter(power);
        runMiddle(power);
    }

    // Run all three intakes together
    public void runBoth(double power) {
        runOuterAndMiddle(power);
        runInner(power);
    }

    public void stop() {
        runBoth(0);
    }

    public double getOuterRPM() {
        return outerIntake.getVelocity() * 60.0 / outerTicksPerRev;
    }

    public double getMiddleRPM() {
        return middleIntake.getVelocity() * 60.0 / middleTicksPerRev;
    }

    public double getInnerRPM() {
        return innerIntake.getVelocity() * 60.0 / innerTicksPerRev;
    }
}
