
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


public class AxonTurretAutoAim {

    // ---------------- SETTINGS ----------------

    private static final double MIN_ANGLE = -90.0;
    private static final double MAX_ANGLE = 90.0;

    private double targetX = 72.0;
    private double targetY = -72.0;

    private double startX = 0.0;
    private double startY = 0.0;
    private double startHeading = 0.0;

    private static final double TURRET_OFFSET_X = -3.25;
    private static final double TURRET_OFFSET_Y = 0.0;

    private static final double AIM_TRIM = 0.0;

    private static final int PWM_MIN = 500;
    private static final int PWM_MAX = 2500;

    private static final double FILTER_TIME = 0.08;
    private static final double DEADBAND = 0.4;

    private static final boolean REVERSE_SERVO = false;


    // ---------------- HARDWARE ----------------

    private GoBildaPinpointDriver pinpoint;
    private ServoImplEx turretServo;

    private boolean initialized = false;

    private final ElapsedTime timer = new ElapsedTime();


    // ---------------- CONTROL STATE ----------------

    private boolean autoAimEnabled = true;
    private boolean filterInitialized = false;


    // ---------------- FILTERED VALUES ----------------

    private double filteredX = 0.0;
    private double filteredY = 0.0;
    private double filteredHeading = 0.0;
    private double filteredTargetAngle = 0.0;


    // ---------------- TURRET VALUES ----------------

    private double commandedAngle = 0.0;
    private double lastServoPosition = -1.0;

    private double pivotX = 0.0;
    private double pivotY = 0.0;

    private double targetDistance = 0.0;
    private double rawTargetAngle = 0.0;

    private boolean targetReachable = false;

    private GoBildaPinpointDriver.DeviceStatus pinpointStatus;


    // =================================================
    // INITIALIZATION
    // =================================================

    public void init(HardwareMap hardwareMap) {

        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class,
                "pinpoint"
        );

        turretServo = hardwareMap.get(
                ServoImplEx.class,
                "turretServo"
        );

        turretServo.setPwmRange(
                new PwmControl.PwmRange(PWM_MIN, PWM_MAX)
        );

        pinpoint.setOffsets(
                -162.56,
                -114.30,
                DistanceUnit.MM
        );

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        pinpoint.resetPosAndIMU();

        commandedAngle = 0.0;
        lastServoPosition = -1.0;

        sendServo(angleToPosition(commandedAngle));

        timer.reset();

        initialized = true;
    }


    // =================================================
    // START
    // =================================================

    public void start() {

        checkInitialized();

        setStartPose();

        timer.reset();
    }


    // =================================================
    // MAIN UPDATE
    // =================================================

    // Call repeatedly inside a TeleOp or Autonomous loop.
    // No gamepad is required.

    public void update() {

        checkInitialized();

        double dt = Range.clip(
                timer.seconds(),
                0.001,
                0.05
        );

        timer.reset();

        pinpoint.update();

        pinpointStatus = pinpoint.getDeviceStatus();

        if (pinpointStatus !=
                GoBildaPinpointDriver.DeviceStatus.READY) {

            return;
        }

        Pose2D pose = pinpoint.getPosition();

        double robotX = pose.getX(DistanceUnit.INCH);
        double robotY = pose.getY(DistanceUnit.INCH);
        double robotHeading = pose.getHeading(AngleUnit.DEGREES);


        // ---------------- POSITION FILTERING ----------------

        if (!filterInitialized) {

            filteredX = robotX;
            filteredY = robotY;
            filteredHeading = robotHeading;

            filterInitialized = true;
        }

        double alpha = dt / (FILTER_TIME + dt);

        filteredX += alpha * (robotX - filteredX);

        filteredY += alpha * (robotY - filteredY);

        filteredHeading = AngleUnit.normalizeDegrees(
                filteredHeading +
                        alpha * AngleUnit.normalizeDegrees(
                                robotHeading - filteredHeading
                        )
        );


        // ---------------- AUTOMATIC AIM ----------------

        if (autoAimEnabled) {

            double headingRad = Math.toRadians(filteredHeading);

            // Calculate turret pivot position.
            pivotX =
                    filteredX +
                            TURRET_OFFSET_X * Math.cos(headingRad) -
                            TURRET_OFFSET_Y * Math.sin(headingRad);

            pivotY =
                    filteredY +
                            TURRET_OFFSET_X * Math.sin(headingRad) +
                            TURRET_OFFSET_Y * Math.cos(headingRad);


            // Calculate distance to target.
            double dx = targetX - pivotX;
            double dy = targetY - pivotY;

            targetDistance = Math.hypot(dx, dy);


            // Calculate field angle.
            double fieldAngle = Math.toDegrees(
                    Math.atan2(dy, dx)
            );


            // Convert field angle to robot-relative angle.
            rawTargetAngle = AngleUnit.normalizeDegrees(
                    fieldAngle - filteredHeading + AIM_TRIM
            );


            // Check turret reachability.
            targetReachable =
                    rawTargetAngle >= MIN_ANGLE &&
                            rawTargetAngle <= MAX_ANGLE;


            if (targetReachable) {

                // Preserve original target-angle filtering.
                if (Math.abs(
                        rawTargetAngle - filteredTargetAngle
                ) > 180.0) {

                    filteredTargetAngle = rawTargetAngle;

                } else {

                    double angleError =
                            AngleUnit.normalizeDegrees(
                                    rawTargetAngle - filteredTargetAngle
                            );

                    filteredTargetAngle =
                            AngleUnit.normalizeDegrees(
                                    filteredTargetAngle + alpha * angleError
                            );
                }


                // Update commanded turret angle.
                double error =
                        AngleUnit.normalizeDegrees(
                                filteredTargetAngle - commandedAngle
                        );

                if (Math.abs(error) > DEADBAND) {
                    commandedAngle = filteredTargetAngle;
                }
            }
        }


        // ---------------- SERVO UPDATE ----------------

        commandedAngle = Range.clip(
                commandedAngle,
                MIN_ANGLE,
                MAX_ANGLE
        );

        double servoPosition = angleToPosition(commandedAngle);

        sendServo(servoPosition);
    }


    // =================================================
    // STARTING POSE
    // =================================================

    public void setStartPose() {

        checkInitialized();

        pinpoint.setPosition(
                new Pose2D(
                        DistanceUnit.INCH,
                        startX,
                        startY,
                        AngleUnit.DEGREES,
                        startHeading
                )
        );

        filterInitialized = false;
    }


    // Set a custom starting pose.
    // Call before start().

    public void setStartPose(
            double x,
            double y,
            double heading
    ) {

        startX = x;
        startY = y;
        startHeading = heading;
    }


    // =================================================
    // TARGET SETTINGS
    // =================================================

    public void setTargetPosition(double x, double y) {

        targetX = x;
        targetY = y;
    }


    // =================================================
    // AUTO AIM SETTINGS
    // =================================================

    public void setAutoAimEnabled(boolean enabled) {

        autoAimEnabled = enabled;
    }


    public boolean isAutoAimEnabled() {

        return autoAimEnabled;
    }


    // =================================================
    // SERVO FUNCTIONS
    // =================================================

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


    // =================================================
    // GETTERS
    // =================================================

    public double getCommandedAngle() {
        return commandedAngle;
    }

    public double getRawTargetAngle() {
        return rawTargetAngle;
    }

    public double getFilteredTargetAngle() {
        return filteredTargetAngle;
    }

    public double getServoPosition() {
        return angleToPosition(commandedAngle);
    }

    public double getTargetDistance() {
        return targetDistance;
    }

    public boolean isTargetReachable() {
        return targetReachable;
    }

    public double getRobotX() {
        return filteredX;
    }

    public double getRobotY() {
        return filteredY;
    }

    public double getRobotHeading() {
        return filteredHeading;
    }

    public double getPivotX() {
        return pivotX;
    }

    public double getPivotY() {
        return pivotY;
    }


    // =================================================
    // TELEMETRY
    // =================================================

    public void addTelemetry(Telemetry telemetry) {

        telemetry.addData(
                "MODE",
                autoAimEnabled ? "AUTO" : "DISABLED"
        );

        telemetry.addData(
                "PINPOINT",
                pinpointStatus
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
                "Servo Position",
                "%.3f",
                getServoPosition()
        );

        if (autoAimEnabled && !targetReachable) {
            telemetry.addLine("TARGET OUTSIDE TURRET RANGE");
        }

        if (pinpointStatus !=
                GoBildaPinpointDriver.DeviceStatus.READY) {

            telemetry.addData(
                    "ERROR",
                    "PINPOINT NOT READY"
            );
        }
    }


    // =================================================
    // INITIALIZATION CHECK
    // =================================================

    private void checkInitialized() {

        if (!initialized) {

            throw new IllegalStateException(
                    "TurretAutoAimSubsystem has not been initialized!"
            );
        }
    }
}
