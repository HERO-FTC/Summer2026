
package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.config.RoboInit;

public class IntakeSubsystemV2 {

    private final DcMotorEx intake;

    private static final double TARGET_RPM = 1150.0 * 0.99;

    private final double ticksPerRev;

    public IntakeSubsystemV2(RoboInit robot) {

        intake = robot.intake;

        ticksPerRev = intake.getMotorType().getTicksPerRev();

        intake.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        intake.setMode(
                DcMotor.RunMode.RUN_USING_ENCODER
        );
    }

    public void run(double power) {

        power = Math.max(-1.0, Math.min(1.0, power));

        intake.setVelocity(
                power * TARGET_RPM * ticksPerRev / 60.0
        );
    }

    public void stop() {
        intake.setVelocity(0);
    }

    public double getRPM() {
        return intake.getVelocity() * 60.0 / ticksPerRev;
    }
}
