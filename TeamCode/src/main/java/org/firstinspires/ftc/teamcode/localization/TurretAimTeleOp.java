package org.firstinspires.ftc.teamcode.localization;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(name = "Turret Auto Aim", group = "Sensor")
public class TurretAimTeleOp extends OpMode {

    static final double SERVO_LEFT_POS   = 0.1;
    static final double SERVO_RIGHT_POS  = 1.0;
    static final double SERVO_CENTER_POS = 0.55;
    static final double TURRET_TRAVEL_DEG = 150.0;

    static final double TURRET_OFFSET_X_IN = -3.25;
    static final double TURRET_OFFSET_Y_IN = 0.0;

    static final double TARGET_X_IN = 57.0;
    static final double TARGET_Y_IN = 50.0;

    static final double START_X_IN = 0.0;
    static final double START_Y_IN = 0.0;
    static final double START_HEADING_DEG = 0.0;

    static final double AIM_TRIM_DEG = 0.0;

    static final double SMOOTHING_TIME_CONSTANT_SEC = 0.03;
    static final double MAX_RATE_DEG_PER_SEC = 300.0;
    static final double DEADBAND_DEG = 0.5;
    static final double SERVO_WRITE_DEADBAND = 0.001;

    GoBildaPinpointDriver pinpoint;
    Servo turret;

    ElapsedTime initTimer = new ElapsedTime();
    ElapsedTime loopTimer = new ElapsedTime();

    boolean poseSet = false;
    boolean autoAim = true;
    boolean prevB = false;
    double manualPos = SERVO_CENTER_POS;
    double currentTurretDeg = 0.0;
    double lastServoPos = Double.NaN;
    String direction = "HOLD";

    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        turret = hardwareMap.get(Servo.class, "turret");

        configurePinpoint();
        initTimer.reset();

        turret.setPosition(SERVO_CENTER_POS);
        lastServoPos = SERVO_CENTER_POS;
        currentTurretDeg = 0.0;
    }

    @Override
    public void init_loop() {
        pinpoint.update();
        if (!poseSet && initTimer.seconds() > 0.6) {
            setStartPose();
            poseSet = true;
        }
        Pose2D pose = pinpoint.getPosition();
        telemetry.addData("Pose set", poseSet ? "yes" : "waiting for IMU calibration");
        telemetry.addData("Pose", "x=%.1f y=%.1f h=%.1f",
                pose.getX(DistanceUnit.INCH),
                pose.getY(DistanceUnit.INCH),
                pose.getHeading(AngleUnit.DEGREES));
        telemetry.update();
    }

    @Override
    public void start() {
        setStartPose();
        poseSet = true;
        loopTimer.reset();
    }

    @Override
    public void loop() {
        double dt = clamp(loopTimer.seconds(), 0.0, 0.1);
        loopTimer.reset();

        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();

        double x = pose.getX(DistanceUnit.INCH);
        double y = pose.getY(DistanceUnit.INCH);
        double headingRad = pose.getHeading(AngleUnit.RADIANS);
        double robotHeadingDeg = Math.toDegrees(headingRad);

        if (gamepad1.b && !prevB) autoAim = !autoAim;
        prevB = gamepad1.b;

        double desiredTurretDeg = 0;
        double fieldAngleDeg = 0;
        double pivotX = x;
        double pivotY = y;
        double servoPos;

        if (autoAim) {
            pivotX = x + TURRET_OFFSET_X_IN * Math.cos(headingRad) - TURRET_OFFSET_Y_IN * Math.sin(headingRad);
            pivotY = y + TURRET_OFFSET_X_IN * Math.sin(headingRad) + TURRET_OFFSET_Y_IN * Math.cos(headingRad);

            fieldAngleDeg = Math.toDegrees(Math.atan2(TARGET_Y_IN - pivotY, TARGET_X_IN - pivotX));
            desiredTurretDeg = normalizeDeg(fieldAngleDeg - robotHeadingDeg + AIM_TRIM_DEG);

            double half = TURRET_TRAVEL_DEG / 2.0;
            double goalDeg = clamp(desiredTurretDeg, -half, half);

            double alpha = SMOOTHING_TIME_CONSTANT_SEC > 0
                    ? 1.0 - Math.exp(-dt / SMOOTHING_TIME_CONSTANT_SEC)
                    : 1.0;
            double wantedStep = (goalDeg - currentTurretDeg) * alpha;
            double maxStep = MAX_RATE_DEG_PER_SEC * dt;
            currentTurretDeg += clamp(wantedStep, -maxStep, maxStep);

            double remaining = goalDeg - currentTurretDeg;
            if (remaining > DEADBAND_DEG) {
                direction = "COUNTER-CLOCKWISE (left)";
            } else if (remaining < -DEADBAND_DEG) {
                direction = "CLOCKWISE (right)";
            } else {
                direction = "HOLD";
            }

            servoPos = angleToServo(currentTurretDeg);
        } else {
            double leftSign = Math.signum(SERVO_LEFT_POS - SERVO_RIGHT_POS);
            if (gamepad1.dpad_left)  manualPos += 0.002 * leftSign;
            if (gamepad1.dpad_right) manualPos -= 0.002 * leftSign;
            manualPos = clamp(manualPos, 0.0, 1.0);
            servoPos = manualPos;
            currentTurretDeg = servoToAngle(manualPos);
            direction = "MANUAL";
        }

        if (Double.isNaN(lastServoPos) || Math.abs(servoPos - lastServoPos) >= SERVO_WRITE_DEADBAND) {
            turret.setPosition(servoPos);
            lastServoPos = servoPos;
        }

        telemetry.addData("Mode (B toggles)", autoAim ? "AUTO AIM" : "MANUAL CALIBRATE (dpad L/R)");
        telemetry.addData("Pose", "x=%.1f y=%.1f h=%.1f", x, y, robotHeadingDeg);
        telemetry.addData("Turret pivot (field)", "x=%.1f y=%.1f", pivotX, pivotY);
        telemetry.addData("Field angle to goal (deg)", "%.1f", fieldAngleDeg);
        telemetry.addData("Desired turret angle (deg)", "%.1f", desiredTurretDeg);
        telemetry.addData("Current turret angle (deg)", "%.1f", currentTurretDeg);
        telemetry.addData("Turning", direction);
        telemetry.addData("Servo position", "%.3f", servoPos);
        telemetry.addData("Within turret range?",
                Math.abs(desiredTurretDeg) <= TURRET_TRAVEL_DEG / 2.0 ? "yes" : "NO (clamped)");
        telemetry.update();
    }

    void setStartPose() {
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, START_X_IN, START_Y_IN,
                AngleUnit.DEGREES, START_HEADING_DEG));
    }

    static double angleToServo(double turretDeg) {
        double half = TURRET_TRAVEL_DEG / 2.0;
        double clamped = clamp(turretDeg, -half, half);
        double pos;
        if (clamped >= 0) {
            pos = SERVO_CENTER_POS + (clamped / half) * (SERVO_LEFT_POS - SERVO_CENTER_POS);
        } else {
            pos = SERVO_CENTER_POS + (-clamped / half) * (SERVO_RIGHT_POS - SERVO_CENTER_POS);
        }
        double lo = Math.min(SERVO_LEFT_POS, SERVO_RIGHT_POS);
        double hi = Math.max(SERVO_LEFT_POS, SERVO_RIGHT_POS);
        return clamp(pos, lo, hi);
    }

    static double servoToAngle(double pos) {
        double half = TURRET_TRAVEL_DEG / 2.0;
        double fromCenter = pos - SERVO_CENTER_POS;
        double leftSpan = SERVO_LEFT_POS - SERVO_CENTER_POS;
        double rightSpan = SERVO_RIGHT_POS - SERVO_CENTER_POS;
        double deg;
        if (leftSpan != 0 && fromCenter * leftSpan >= 0) {
            deg = (fromCenter / leftSpan) * half;
        } else if (rightSpan != 0) {
            deg = -(fromCenter / rightSpan) * half;
        } else {
            deg = 0;
        }
        return clamp(deg, -half, half);
    }

    static double normalizeDeg(double deg) {
        return deg - 360.0 * Math.floor((deg + 180.0) / 360.0);
    }

    static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    void configurePinpoint() {
        pinpoint.setOffsets(-162.56, -114.30, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();
    }
}