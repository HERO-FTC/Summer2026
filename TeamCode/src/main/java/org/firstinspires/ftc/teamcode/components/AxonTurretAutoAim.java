package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Axon Turret Auto Aim V2", group = "Turret")
public class AxonTurretAutoAim extends OpMode {

    static final double MIN_ANGLE = -90.0;
    static final double MAX_ANGLE = 90.0;

    private static final double TARGET_X = 72.0;
    private static final double TARGET_Y = -72.0;

    private static final double START_X = 0.0;
    private static final double START_Y = 0.0;
    private static final double START_HEADING = 0.0;

    private static final double TURRET_OFFSET_X = -3.25;
    private static final double TURRET_OFFSET_Y = 0.0;

    private static final double AIM_TRIM = 0.0;

    static final int PWM_MIN = 500;
    static final int PWM_MAX = 2500;

    private static final double FILTER_TIME = 0.08;
    private static final double DEADBAND = 0.4;
    private static final double MANUAL_SPEED = 90.0;

    private static final boolean REVERSE_SERVO = false;

    private GoBildaPinpointDriver pinpoint;
    private ServoImplEx turretServo;

    private final ElapsedTime timer = new ElapsedTime();

    private boolean autoAimEnabled = true;
    private boolean lastB = false;
    private boolean lastX = false;

    private boolean filterInitialized = false;

    private double filteredX = 0.0;
    private double filteredY = 0.0;
    private double filteredHeading = 0.0;
    private double filteredTargetAngle = 0.0;

    private double commandedAngle = 0.0;
    private double lastServoPosition = -1.0;

    private double pivotX = 0.0;
    private double pivotY = 0.0;
    private double targetDistance = 0.0;
    private double rawTargetAngle = 0.0;

    private boolean targetReachable = false;

    @Override
    public void init() {

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

        sendServo(angleToPosition(commandedAngle));

        timer.reset();

        telemetry.addLine("AXON TURRET READY");
        telemetry.addData("Travel", "180 degrees");
        telemetry.addData("PWM", "%d - %d", PWM_MIN, PWM_MAX);
        telemetry.update();
    }

    @Override
    public void init_loop() {

        pinpoint.update();

        telemetry.addData(
                "Pinpoint",
                pinpoint.getDeviceStatus()
        );

        telemetry.update();
    }

    @Override
    public void start() {

        setStartPose();
        timer.reset();
    }

    @Override
    public void loop() {

        double dt = Range.clip(
                timer.seconds(),
                0.001,
                0.05
        );

        timer.reset();

        pinpoint.update();

        if (pinpoint.getDeviceStatus() !=
                GoBildaPinpointDriver.DeviceStatus.READY) {

            telemetry.addData("ERROR", "PINPOINT NOT READY");
            telemetry.addData("Status", pinpoint.getDeviceStatus());
            telemetry.update();
            return;
        }

        Pose2D pose = pinpoint.getPosition();

        double robotX = pose.getX(DistanceUnit.INCH);
        double robotY = pose.getY(DistanceUnit.INCH);
        double robotHeading = pose.getHeading(AngleUnit.DEGREES);

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

        if (gamepad1.b && !lastB) {
            autoAimEnabled = !autoAimEnabled;
        }

        lastB = gamepad1.b;

        if (gamepad1.x && !lastX) {
            setStartPose();
        }

        lastX = gamepad1.x;

        if (autoAimEnabled) {

            double headingRad = Math.toRadians(filteredHeading);

            pivotX =
                    filteredX +
                            TURRET_OFFSET_X * Math.cos(headingRad) -
                            TURRET_OFFSET_Y * Math.sin(headingRad);

            pivotY =
                    filteredY +
                            TURRET_OFFSET_X * Math.sin(headingRad) +
                            TURRET_OFFSET_Y * Math.cos(headingRad);

            double dx = TARGET_X - pivotX;
            double dy = TARGET_Y - pivotY;

            targetDistance = Math.hypot(dx, dy);

            double fieldAngle = Math.toDegrees(
                    Math.atan2(dy, dx)
            );

            rawTargetAngle = AngleUnit.normalizeDegrees(
                    fieldAngle - filteredHeading + AIM_TRIM
            );

            targetReachable =
                    rawTargetAngle >= MIN_ANGLE &&
                            rawTargetAngle <= MAX_ANGLE;

            if (targetReachable) {

                if (Math.abs(rawTargetAngle - filteredTargetAngle) > 180.0) {
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

                double error =
                        AngleUnit.normalizeDegrees(
                                filteredTargetAngle - commandedAngle
                        );

                if (Math.abs(error) > DEADBAND) {
                    commandedAngle = filteredTargetAngle;
                }

            }

        } else {

            targetReachable = true;

            if (gamepad1.dpad_left) {
                commandedAngle += MANUAL_SPEED * dt;
            }

            if (gamepad1.dpad_right) {
                commandedAngle -= MANUAL_SPEED * dt;
            }

            commandedAngle = Range.clip(
                    commandedAngle,
                    MIN_ANGLE,
                    MAX_ANGLE
            );

            filteredTargetAngle = commandedAngle;
        }

        commandedAngle = Range.clip(
                commandedAngle,
                MIN_ANGLE,
                MAX_ANGLE
        );

        double servoPosition = angleToPosition(commandedAngle);

        sendServo(servoPosition);

        telemetry.addData(
                "MODE",
                autoAimEnabled ? "AUTO" : "MANUAL"
        );

        telemetry.addData(
                "PINPOINT",
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
                "Servo Position",
                "%.3f",
                servoPosition
        );

        if (autoAimEnabled && !targetReachable) {
            telemetry.addLine("TARGET OUTSIDE TURRET RANGE");
        }

        telemetry.update();
    }

    private void setStartPose() {

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
    }

    private void sendServo(double position) {

        if (Math.abs(position - lastServoPosition) >= 0.001) {

            turretServo.setPosition(position);

            lastServoPosition = position;
        }
    }

    static double angleToPosition(double angle) {

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
