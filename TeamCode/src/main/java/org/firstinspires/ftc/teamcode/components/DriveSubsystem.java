package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.config.RobotInit;
import org.jetbrains.annotations.UnknownNullability;

public class DriveSubsystem {

    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;

    private static final double DEFAULT_POWER = 0.70;
    private static final double MAX_POWER = 0.90;
    private static final double MIN_POWER = 0.50;

    public DriveSubsystem(@UnknownNullability RobotInit robot) {
        frontLeft = robot.frontLeft;
        frontRight = robot.frontRight;
        backLeft = robot.backLeft;
        backRight = robot.backRight;

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void drive(double forward, double strafe, double turn) {
        drive(forward, strafe, turn, 0, 0);
    }

    public void drive(double forward, double strafe, double turn,
                      double rightTrigger, double leftTrigger) {

        double driveScale = DEFAULT_POWER;

        if (rightTrigger > 0.1) {
            driveScale = MAX_POWER;
        }

        if (leftTrigger > 0.1) {
            driveScale = MIN_POWER;
        }

        // SWAPPED FORWARD & STRAFE MIXING TO FIX THE STRAFING BUG:
        // Standard mecanum math maps strafe and forward depending on roller orientation.
        // If forward inputs caused strafing, swapping them aligns input axes correctly:
        double fl = strafe + forward + turn;
        double fr = strafe - forward - turn;
        double bl = strafe - forward + turn;
        double br = strafe + forward - turn;

        double max = Math.max(
                1.0,
                Math.max(
                        Math.max(Math.abs(fl), Math.abs(fr)),
                        Math.max(Math.abs(bl), Math.abs(br))
                )
        );

        fl = (fl / max) * driveScale;
        fr = (fr / max) * driveScale;
        bl = (bl / max) * driveScale;
        br = (br / max) * driveScale;

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}