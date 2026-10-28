
package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.config.RobotInit;

public class DriveSubsystem {

    private final DcMotor frontLeft;
    private final DcMotor frontRight;
    private final DcMotor backLeft;
    private final DcMotor backRight;

    private static final double DEFAULT_POWER = 0.70;
    private static final double MAX_POWER = 0.90;
    private static final double MIN_POWER = 0.50;

    public DriveSubsystem(RobotInit config) {
        frontLeft = config.frontLeft;
        frontRight = config.frontRight;
        backLeft = config.backLeft;
        backRight = config.backRight;

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive(double forward, double strafe, double turn,
                      double rightTrigger, double leftTrigger) {

        double powerLimit = DEFAULT_POWER;

        if (rightTrigger > 0.5) {
            powerLimit = MAX_POWER;
        } else if (leftTrigger > 0.5) {
            powerLimit = MIN_POWER;
        }

        double fl = forward + strafe + turn;
        double fr = forward - strafe - turn;
        double bl = forward - strafe + turn;
        double br = forward + strafe - turn;

        double max = Math.max(
                1.0,
                Math.max(
                        Math.max(Math.abs(fl), Math.abs(fr)),
                        Math.max(Math.abs(bl), Math.abs(br))
                )
        );

        frontLeft.setPower((fl / max) * powerLimit);
        frontRight.setPower((fr / max) * powerLimit);
        backLeft.setPower((bl / max) * powerLimit);
        backRight.setPower((br / max) * powerLimit);
    }

    // Keeps compatibility with existing TeleOp calls.
    public void drive(double forward, double strafe, double turn) {
        drive(forward, strafe, turn, 0.0, 0.0);
    }

    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
