package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.config.RoboInit;

public class IntakeSubsystemV2 {

    private final DcMotorEx intake;
    private final DcMotorEx intake2;

    private static final double TARGET_RPM = 435.0 * 0.99;

    private final double ticksPerRev;
    private final double ticksPerRev2;

    public IntakeSubsystemV2(RoboInit robot, HardwareMap hardwareMap) {

        intake = robot.intake;
        intake2 = hardwareMap.get(DcMotorEx.class, "intake2");

        ticksPerRev = intake.getMotorType().getTicksPerRev();
        ticksPerRev2 = intake2.getMotorType().getTicksPerRev();

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void run(double power) {

        power = Math.max(-1.0, Math.min(1.0, power));

        intake.setVelocity(
                power * TARGET_RPM * ticksPerRev / 60.0
        );

        intake2.setVelocity(
                power * TARGET_RPM * ticksPerRev2 / 60.0
        );
    }

    public void stop() {
        intake.setVelocity(0);
        intake2.setVelocity(0);
    }

    public double getRPM() {
        return intake.getVelocity() * 60.0 / ticksPerRev;
    }

    public double getRPM2() {
        return intake2.getVelocity() * 60.0 / ticksPerRev2;
    }
}