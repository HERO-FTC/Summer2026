
package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class M2TurretAutoAim {

    private static final double MIN_ANGLE = -90.0;
    private static final double MAX_ANGLE = 90.0;

    private static final double TARGET_X = 50.75;
    private static final double TARGET_Y = 70.75;

    private static final double START_X = 0.0;
    private static final double START_Y = 0.0;
    private static final double START_HEADING = 0.0;

    private static final double TURRET_OFFSET_X = 2.0;
    private static final double TURRET_OFFSET_Y = 3.5;

    private static final double AIM_TRIM = 0.0;

    private static final int PWM_MIN = 500;
    private static final int PWM_MAX = 2500;

    private static final double FILTER_TIME = 0.02;

    private static final boolean REVERSE_SERVO = true;

    private final GoBildaPinpointDriver pinpoint;
    private final ServoImplEx turretServo;

    private final ElapsedTime timer = new ElapsedTime();

    private boolean filterInitialized = false;

    private double filteredX = 0.0;
    private double filteredY = 0.0;

    private double commandedAngle = 0.0;
    private double lastServoPosition = -1.0;

    private double pivotX = 0.0;
    private double pivotY = 0.0;

    private double fieldAngle = 0.0;
    private double rawTargetAngle = 0.0;
    private double targetDistance = 0.0;

    private boolean targetReachable = false;

    public M2TurretAutoAim(HardwareMap hardwareMap) {

        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class,
                "pinpoint"
        );

        turretServo = hardwareMap.get(
                ServoImplEx.class,
                "turretAim"
        );

        turretServo.setPwmRange(
                new PwmControl.PwmRange(PWM_MIN, PWM_MAX)
        );

        // Original Pinpoint configuration
        pinpoint.setOffsets(
                2.5,
                -4.75,
                DistanceUnit.INCH
        );

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        pinpoint.resetPosAndIMU();

        commandedAngle = 0.0;
        sendServo(angleToPosition(commandedAngle));

        timer.reset();
    }

    public void start() {

        pinpoint.setPosition(
                new Pose2D(
                        DistanceUnit.INCH,
                        START_X,
                        START_Y,
                        AngleUnit.DEGREES,
                        START_HEADING
                )
        );

        filterInitialized = false;
        timer.reset();
    }

    public void update(Telemetry telemetry) {

        double dt = Range.clip(
                timer.seconds(),
                0.001,
                0.05
        );

        timer.reset();

        pinpoint.update();

        if (pinpoint.getDeviceStatus() !=
                GoBildaPinpointDriver.DeviceStatus.READY) {

            telemetry.addData("Turret Auto Aim", "WAITING FOR PINPOINT");
            telemetry.addData("Pinpoint Status", pinpoint.getDeviceStatus());
            return;
        }

        Pose2D pose = pinpoint.getPosition();

        double robotX = pose.getX(DistanceUnit.INCH);
        double robotY = pose.getY(DistanceUnit.INCH);

        // Current robot heading
        double robotHeading = pose.getHeading(AngleUnit.DEGREES);

        if (!filterInitialized) {
            filteredX = robotX;
            filteredY = robotY;
            filterInitialized = true;
        }

        // Filter robot position
        double alpha = dt / (FILTER_TIME + dt);

        filteredX += alpha * (robotX - filteredX);
        filteredY += alpha * (robotY - filteredY);

        // Convert heading to radians
        double headingRad = Math.toRadians(robotHeading);

        // Calculate turret pivot using the same heading convention
        // as the rotation compensation below.
        pivotX =
                filteredX +
                        TURRET_OFFSET_X * Math.cos(headingRad) +
                        TURRET_OFFSET_Y * Math.sin(headingRad);

        pivotY =
                filteredY -
                        TURRET_OFFSET_X * Math.sin(headingRad) +
                        TURRET_OFFSET_Y * Math.cos(headingRad);

        // Find the field target relative to the turret pivot
        double dx = TARGET_X - pivotX;
        double dy = TARGET_Y - pivotY;

        targetDistance = Math.hypot(dx, dy);

        // Angle to target in field coordinates
        fieldAngle = Math.toDegrees(
                Math.atan2(dy, dx)
        );

        // Convert field angle into robot-relative turret angle
        // This compensates for robot rotation.
        rawTargetAngle = AngleUnit.normalizeDegrees(
                fieldAngle + robotHeading + AIM_TRIM
        );

        targetReachable =
                rawTargetAngle >= MIN_ANGLE &&
                        rawTargetAngle <= MAX_ANGLE;

        if (targetReachable) {
            commandedAngle = rawTargetAngle;
        }

        commandedAngle = Range.clip(
                commandedAngle,
                MIN_ANGLE,
                MAX_ANGLE
        );

        double servoPosition = angleToPosition(commandedAngle);

        sendServo(servoPosition);

        // Telemetry
        telemetry.addData("Turret Mode", "AUTOMATIC");
        telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());

        telemetry.addData("Robot X", "%.2f", robotX);
        telemetry.addData("Robot Y", "%.2f", robotY);

        telemetry.addData("Robot Heading", "%.2f", robotHeading);

        telemetry.addData("Field Target Angle", "%.2f", fieldAngle);
        telemetry.addData("Required Turret Angle", "%.2f", rawTargetAngle);

        telemetry.addData("Turret Pivot X", "%.2f", pivotX);
        telemetry.addData("Turret Pivot Y", "%.2f", pivotY);

        telemetry.addData("Target Distance", "%.2f", targetDistance);

        telemetry.addData("Target Reachable", targetReachable);
        telemetry.addData("Commanded Turret Angle", "%.2f", commandedAngle);
        telemetry.addData("Servo Position", "%.3f", servoPosition);

        if (!targetReachable) {
            telemetry.addLine("TARGET OUTSIDE TURRET RANGE");
        }
    }

    public void stop() {
        sendServo(angleToPosition(commandedAngle));
    }

    private void sendServo(double position) {

        if (Math.abs(position - lastServoPosition) >= 0.001) {
            turretServo.setPosition(position);
            lastServoPosition = position;
        }
    }

    private double angleToPosition(double angle) {

        double clippedAngle = Range.clip(
                angle,
                MIN_ANGLE,
                MAX_ANGLE
        );

        double position =
                (clippedAngle - MIN_ANGLE) /
                        (MAX_ANGLE - MIN_ANGLE);

        if (REVERSE_SERVO) {
            position = 1.0 - position;
        }

        return Range.clip(position, 0.0, 1.0);
    }
}
