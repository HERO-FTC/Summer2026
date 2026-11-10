
package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.TurretTargetConstants;

public class TurretServoController {

    private final Servo turret;

    private double currentAngle;

    public TurretServoController(HardwareMap hardwareMap) {
        turret = hardwareMap.get(Servo.class, "turret");
        currentAngle = 0.0;
        turret.setPosition(TurretTargetConstants.SERVO_CENTER);
    }

    public void setAngle(double angle) {
        double limitedAngle = Math.max(
                -TurretTargetConstants.MAX_TURRET_ANGLE,
                Math.min(TurretTargetConstants.MAX_TURRET_ANGLE, angle)
        );

        double halfRange =
                (TurretTargetConstants.SERVO_MAX
                        - TurretTargetConstants.SERVO_MIN) / 2.0;

        double position =
                TurretTargetConstants.SERVO_CENTER
                        + (limitedAngle
                        / TurretTargetConstants.MAX_TURRET_ANGLE)
                        * halfRange
                        * TurretTargetConstants.SERVO_DIRECTION;

        position = Math.max(
                TurretTargetConstants.SERVO_MIN,
                Math.min(TurretTargetConstants.SERVO_MAX, position)
        );

        turret.setPosition(position);
        currentAngle = limitedAngle;
    }

    public void center() {
        setAngle(0.0);
    }

    public double getCurrentAngle() {
        return currentAngle;
    }

    public double getServoPosition() {
        return turret.getPosition();
    }
}
