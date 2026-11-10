
package org.firstinspires.ftc.teamcode.components;

import org.firstinspires.ftc.teamcode.config.TurretTargetConstants;

public class TurretAlignmentSubsystem {

    private final CustomMecanumLocalizer localizer;
    private final TurretServoController servoController;

    private double targetAngle;
    private double targetDistance;
    private double targetBearing;

    public TurretAlignmentSubsystem(
            CustomMecanumLocalizer localizer,
            TurretServoController servoController
    ) {
        this.localizer = localizer;
        this.servoController = servoController;
    }

    public void update() {
        double dx = TurretTargetConstants.TARGET_X - localizer.getX();
        double dy = TurretTargetConstants.TARGET_Y - localizer.getY();

        targetDistance = Math.hypot(dx, dy);

        targetBearing = Math.toDegrees(Math.atan2(dy, dx));

        targetAngle = normalizeDegrees(
                targetBearing - localizer.getHeadingDegrees()
        );

        if (Math.abs(targetAngle) <= TurretTargetConstants.AIM_DEADBAND) {
            targetAngle = 0.0;
        }

        servoController.setAngle(targetAngle);
    }

    private double normalizeDegrees(double angle) {
        while (angle > 180.0) {
            angle -= 360.0;
        }

        while (angle < -180.0) {
            angle += 360.0;
        }

        return angle;
    }

    public double getTargetAngle() {
        return targetAngle;
    }

    public double getTargetDistance() {
        return targetDistance;
    }

    public double getTargetBearing() {
        return targetBearing;
    }

    public double getRobotX() {
        return localizer.getX();
    }

    public double getRobotY() {
        return localizer.getY();
    }

    public double getRobotHeading() {
        return localizer.getHeadingDegrees();
    }

    public void centerTurret() {
        servoController.center();
    }
}
