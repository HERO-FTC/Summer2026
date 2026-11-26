
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

    // TURRET ANGLE LIMITS
    private static final double MIN_ANGLE = -90.0;
    private static final double MAX_ANGLE = 90.0;

    // TARGET LOCATION
    private static final double TARGET_X = 58.75;
    private static final double TARGET_Y = 70.75;

    // STARTING ROBOT POSITION
    private static final double START_X = 0.0;
    private static final double START_Y = 0.0;
    private static final double START_HEADING = 0.0;

    // TURRET PIVOT OFFSET
    private static final double TURRET_OFFSET_X = 2.0;
    private static final double TURRET_OFFSET_Y = 3.5;

    // AIMING ADJUSTMENT
    private static final double AIM_TRIM = 0.0;

    // SERVO PWM RANGE
    private static final int PWM_MIN = 500;
    private static final int PWM_MAX = 2500;

    // FILTERING
    private static final double FILTER_TIME = 0.02;
    private static final double DEADBAND = 0.2;

    // SERVO DIRECTION
    private static final boolean REVERSE_SERVO = true;

    // HARDWARE
    private final GoBildaPinpointDriver pinpoint;
    private final ServoImplEx turretServo;

    // TIMER
    private final ElapsedTime timer = new ElapsedTime();

    // FILTER STATE
    private boolean filterInitialized = false;

    private double filteredX = 0.0;
    private double filteredY = 0.0;
    private double filteredHeading = 0.0;
    private double filteredTargetAngle = 0.0;

    // TURRET STATE
    private double commandedAngle = 0.0;
    private double lastServoPosition = -1.0;

    // CALCULATED VALUES
    private double pivotX = 0.0;
    private double pivotY = 0.0;
    private double targetDistance = 0.0;
    private double rawTargetAngle = 0.0;

    private boolean targetReachable = false;

    public M2TurretAutoAim(HardwareMap hardwareMap) {

        // INITIALIZE PINPOINT
        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class,
                "pinpoint"
        );

        // INITIALIZE TURRET SERVO
        turretServo = hardwareMap.get(
                ServoImplEx.class,
                "turretAim"
        );

        // CONFIGURE SERVO PWM
        turretServo.setPwmRange(
                new PwmControl.PwmRange(PWM_MIN, PWM_MAX)
        );

        // CONFIGURE PINPOINT OFFSETS
        pinpoint.setOffsets(
                2.5,
                -4.75,
                DistanceUnit.INCH
        );

        // ENCODER RESOLUTION
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        // ENCODER DIRECTIONS
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.REVERSED,
                GoBildaPinpointDriver.EncoderDirection.REVERSED
        );

        // RESET PINPOINT
        pinpoint.resetPosAndIMU();

        // INITIAL TURRET POSITION
        commandedAngle = 0.0;
        sendServo(angleToPosition(commandedAngle));

        timer.reset();
    }

    // CALL ONCE AFTER WAIT FOR START
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

    // CALL THIS EVERY MAIN TELEOP LOOP
    public void update(Telemetry telemetry) {

        // CALCULATE LOOP TIME
        double dt = Range.clip(
                timer.seconds(),
                0.001,
                0.05
        );

        timer.reset();

        // UPDATE PINPOINT
        pinpoint.update();

        // CHECK PINPOINT STATUS
        if (pinpoint.getDeviceStatus() !=
                GoBildaPinpointDriver.DeviceStatus.READY) {

            telemetry.addData("Turret Auto Aim", "WAITING FOR PINPOINT");
            telemetry.addData("Pinpoint Status", pinpoint.getDeviceStatus());
            return;
        }

        // GET ROBOT POSITION
        Pose2D pose = pinpoint.getPosition();

        double robotX = pose.getX(DistanceUnit.INCH);
        double robotY = pose.getY(DistanceUnit.INCH);

        double robotHeading = pose.getHeading(
                AngleUnit.DEGREES
        );

        // INITIALIZE FILTER
        if (!filterInitialized) {

            filteredX = robotX;
            filteredY = robotY;
            filteredHeading = robotHeading;

            filterInitialized = true;
        }

        // FILTERING
        double alpha = dt / (FILTER_TIME + dt);

        filteredX += alpha * (robotX - filteredX);
        filteredY += alpha * (robotY - filteredY);

        filteredHeading = AngleUnit.normalizeDegrees(
                filteredHeading +
                        alpha * AngleUnit.normalizeDegrees(
                                robotHeading - filteredHeading
                        )
        );

        // CALCULATE TURRET PIVOT LOCATION
        double headingRad = Math.toRadians(filteredHeading);

        pivotX =
                filteredX +
                        TURRET_OFFSET_X * Math.cos(headingRad) -
                        TURRET_OFFSET_Y * Math.sin(headingRad);

        pivotY =
                filteredY +
                        TURRET_OFFSET_X * Math.sin(headingRad) +
                        TURRET_OFFSET_Y * Math.cos(headingRad);

        // CALCULATE TARGET DISTANCE
        double dx = TARGET_X - pivotX;
        double dy = TARGET_Y - pivotY;

        targetDistance = Math.hypot(dx, dy);

        // CALCULATE FIELD ANGLE
        double fieldAngle = Math.toDegrees(
                Math.atan2(dy, dx)
        );

        // CALCULATE REQUIRED TURRET ANGLE
        rawTargetAngle = AngleUnit.normalizeDegrees(
                fieldAngle - filteredHeading + AIM_TRIM
        );

        // CHECK TURRET RANGE
        targetReachable =
                rawTargetAngle >= MIN_ANGLE &&
                        rawTargetAngle <= MAX_ANGLE;

        // FILTER TARGET ANGLE
        if (targetReachable) {

            if (Math.abs(
                    rawTargetAngle - filteredTargetAngle
            ) > 180.0) {

                filteredTargetAngle = rawTargetAngle;

            } else {

                double angleError = AngleUnit.normalizeDegrees(
                        rawTargetAngle - filteredTargetAngle
                );

                filteredTargetAngle = AngleUnit.normalizeDegrees(
                        filteredTargetAngle + alpha * angleError
                );
            }

            // CALCULATE ANGLE ERROR
            double error = AngleUnit.normalizeDegrees(
                    filteredTargetAngle - commandedAngle
            );

            // APPLY DEADBAND
            if (Math.abs(error) > DEADBAND) {
                commandedAngle = filteredTargetAngle;
            }
        }

        // SERVO CONTROL
        commandedAngle = Range.clip(
                commandedAngle,
                MIN_ANGLE,
                MAX_ANGLE
        );

        double servoPosition = angleToPosition(commandedAngle);

        sendServo(servoPosition);

        // TELEMETRY
        telemetry.addData("Turret Mode", "AUTOMATIC");

        telemetry.addData(
                "Pinpoint",
                pinpoint.getDeviceStatus()
        );

        telemetry.addData(
                "Robot X",
                "%.2f",
                filteredX
        );

        telemetry.addData(
                "Robot Y",
                "%.2f",
                filteredY
        );

        telemetry.addData(
                "Heading",
                "%.2f",
                filteredHeading
        );

        telemetry.addData(
                "Raw Target Angle",
                "%.2f",
                rawTargetAngle
        );

        telemetry.addData(
                "Filtered Target Angle",
                "%.2f",
                filteredTargetAngle
        );

        telemetry.addData(
                "Turret Pivot",
                "X %.2f | Y %.2f",
                pivotX,
                pivotY
        );

        telemetry.addData(
                "Target Distance",
                "%.2f inches",
                targetDistance
        );

        telemetry.addData(
                "Target Reachable",
                targetReachable
        );

        telemetry.addData(
                "Turret Angle",
                "%.2f",
                commandedAngle
        );

        telemetry.addData(
                "Turret Servo Position",
                "%.3f",
                servoPosition
        );

        if (!targetReachable) {
            telemetry.addLine("TARGET OUTSIDE TURRET RANGE");
        }
    }

    // SHUTDOWN METHOD
    // KEEP TURRET AT LAST COMMANDED POSITION
    public void stop() {

        sendServo(angleToPosition(commandedAngle));
    }

    // SEND POSITION TO SERVO
    private void sendServo(double position) {

        if (Math.abs(
                position - lastServoPosition
        ) >= 0.001) {

            turretServo.setPosition(position);
            lastServoPosition = position;
        }
    }

    // CONVERT ANGLE TO SERVO POSITION
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
