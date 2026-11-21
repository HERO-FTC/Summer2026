
package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.config.RoboInit;

public class IntakeSubsystemV2 {

    private final DcMotorEx intake;

    public IntakeSubsystemV2(RoboInit robot) {

        intake = robot.intake;

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void run(double power) {
        intake.setPower(power);
    }

    public void stop() {
        intake.setPower(0);
    }

    public double getRPM() {
        return intake.getVelocity() * 60.0 / 28.0;
    }
}
