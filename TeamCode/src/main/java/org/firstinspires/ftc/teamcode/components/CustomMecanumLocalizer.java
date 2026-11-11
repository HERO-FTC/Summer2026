
package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.TurretTargetConstants;

public class CustomMecanumLocalizer {

    private final DcMotorEx frontLeft;
    private final DcMotorEx frontRight;
    private final DcMotorEx backLeft;
    private final DcMotorEx backRight;
    private final IMU imu;

    private int lastFL;
    private int lastFR;
    private int lastBL;
    private int lastBR;

    private double x;
    private double y;
    private double heading;
    private double headingOffset;

    private final double inchesPerTick;

    public CustomMecanumLocalizer(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot orientation =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                );

        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();

        double ticksPerRev =
                frontLeft.getMotorType().getTicksPerRev();

        inchesPerTick =
                Math.PI * TurretTargetConstants.WHEEL_DIAMETER / ticksPerRev;

        x = TurretTargetConstants.START_X;
        y = TurretTargetConstants.START_Y;

        headingOffset =
                Math.toRadians(TurretTargetConstants.START_HEADING);

        heading = headingOffset;

        resetEncoderPositions();
    }

    public void update() {
        int currentFL = frontLeft.getCurrentPosition();
        int currentFR = frontRight.getCurrentPosition();
        int currentBL = backLeft.getCurrentPosition();
        int currentBR = backRight.getCurrentPosition();

        double dFL = (currentFL - lastFL) * inchesPerTick
                * TurretTargetConstants.FL_DIRECTION;

        double dFR = (currentFR - lastFR) * inchesPerTick
                * TurretTargetConstants.FR_DIRECTION;

        double dBL = (currentBL - lastBL) * inchesPerTick
                * TurretTargetConstants.BL_DIRECTION;

        double dBR = (currentBR - lastBR) * inchesPerTick
                * TurretTargetConstants.BR_DIRECTION;

        lastFL = currentFL;
        lastFR = currentFR;
        lastBL = currentBL;
        lastBR = currentBR;

        double forward =
                (dFL + dFR + dBL + dBR) / 4.0;

        double right =
                (dFL - dFR - dBL + dBR) / 4.0
                        * TurretTargetConstants.STRAFE_MULTIPLIER;

        double newHeading =
                headingOffset
                        + imu.getRobotYawPitchRollAngles()
                        .getYaw(AngleUnit.RADIANS);

        double headingDelta =
                normalizeRadians(newHeading - heading);

        double midpointHeading =
                heading + headingDelta / 2.0;

        double fieldX =
                forward * Math.cos(midpointHeading)
                        + right * Math.sin(midpointHeading);

        double fieldY =
                forward * Math.sin(midpointHeading)
                        - right * Math.cos(midpointHeading);

        x += fieldX;
        y += fieldY;
        heading = newHeading;
    }

    public void setPose(
            double newX,
            double newY,
            double newHeadingDegrees
    ) {
        x = newX;
        y = newY;

        headingOffset = Math.toRadians(newHeadingDegrees);

        imu.resetYaw();

        heading = headingOffset;

        resetEncoderPositions();
    }

    private void resetEncoderPositions() {
        lastFL = frontLeft.getCurrentPosition();
        lastFR = frontRight.getCurrentPosition();
        lastBL = backLeft.getCurrentPosition();
        lastBR = backRight.getCurrentPosition();
    }

    private double normalizeRadians(double angle) {
        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }

        while (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getHeadingDegrees() {
        return Math.toDegrees(heading);
    }
}
