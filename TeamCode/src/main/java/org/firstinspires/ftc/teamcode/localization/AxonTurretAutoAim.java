package org.firstinspires.ftc.teamcode.localization;

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

@TeleOp(name = "Axon Turret Only", group = "Turret")
public class AxonTurretAutoAim extends OpMode {

    // Turret range of motion: 180 degrees total (+90 left, -90 right)
    static final double LEFT_LIMIT_ANGLE_DEG  =  90.0;
    static final double RIGHT_LIMIT_ANGLE_DEG = -90.0;
    static final double LEFT_LIMIT_SERVO_POS  =  1.00;
    static final double RIGHT_LIMIT_SERVO_POS =  0.00;

    // PWM window that makes servo 0.0 -> 1.0 equal ~180 deg.
    // Measured on the robot: a ~1014 us window only gave ~90 deg, so this Axon is on a
    // ~180 deg range (about 11.3 us/deg). The full 500-2500 us window gives ~177 deg.
    // If the turret travels MORE than 180 deg, shrink this window; if LESS, it is already
    // at the hardware limit, so reprogram the Axon's range instead.
    static final int PWM_MIN_US = 500;
    static final int PWM_MAX_US = 2500;

    static final double TARGET_X_IN = 72.0;
    static final double TARGET_Y_IN = -72.0;

    static final double START_X_IN = 0.0;
    static final double START_Y_IN = 0.0;
    static final double START_HEADING_DEG = 0.0;

    static final double TURRET_OFFSET_X_IN = -3.25;
    static final double TURRET_OFFSET_Y_IN = 0.0;
    static final double AIM_TRIM_DEG = 0.0;

    // Low-pass time constant (seconds). Higher = smoother but laggier.
    // Pinpoint pose is already clean, so keep this small for a fast response.
    static final double SMOOTHING_TAU_SEC = 0.02;
    // Max turret speed in deg/sec. Prevents sudden snaps.
    static final double MAX_SLEW_DEG_PER_SEC = 360.0;
    // Ignore angle errors smaller than this (deg) so the servo stops hunting.
    static final double DEADBAND_DEG = 1.0;
    // Only send a new servo position if it changed by at least this much (0.005 = ~0.9 deg).
    static final double SERVO_WRITE_TOLERANCE = 0.005;
    // Manual D-pad speed in deg/sec.
    static final double MANUAL_SPEED_DEG_PER_SEC = 60.0;
    // Stop updating the aim when the turret pivot is this close to the target (atan2 gets unstable).
    static final double MIN_AIM_DISTANCE_IN = 6.0;

    private GoBildaPinpointDriver pinpoint;
    private ServoImplEx turretServo;

    private final ElapsedTime loopTimer = new ElapsedTime();
    private boolean poseInitialized = false;
    private boolean autoAimEnabled = true;
    private boolean lastBButton = false;

    // The turret angle we are actually commanding (smoothed).
    private double commandedAngleDeg = 0.0;
    private double lastSentServoPos = -1.0;

    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        turretServo = hardwareMap.get(ServoImplEx.class, "turretServo");
        // Make servo positions 0.0 -> 1.0 cover ~180 degrees of real travel.
        turretServo.setPwmRange(new PwmControl.PwmRange(PWM_MIN_US, PWM_MAX_US));

        pinpoint.setOffsets(-162.56, -114.30, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        pinpoint.resetPosAndIMU();

        commandedAngleDeg = 0.0;
        sendServo(angleToServoPos(commandedAngleDeg));
    }

    @Override
    public void init_loop() {
        pinpoint.update();

        if (!poseInitialized && pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY) {
            setStartPose();
            poseInitialized = true;
        }

        telemetry.addData("Pinpoint Status", pinpoint.getDeviceStatus());
        telemetry.addData("Ready to Start?", poseInitialized ? "YES" : "NO - Calibrating IMU...");
        telemetry.update();
    }

    @Override
    public void start() {
        if (!poseInitialized) {
            setStartPose();
            poseInitialized = true;
        }
        loopTimer.reset();
    }

    @Override
    public void loop() {
        double dt = Range.clip(loopTimer.seconds(), 0.0001, 0.1);
        loopTimer.reset();

        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();
        double robotX = pose.getX(DistanceUnit.INCH);
        double robotY = pose.getY(DistanceUnit.INCH);
        double headingRad = pose.getHeading(AngleUnit.RADIANS);
        double headingDeg = pose.getHeading(AngleUnit.DEGREES);

        if (gamepad1.b && !lastBButton) {
            autoAimEnabled = !autoAimEnabled;
        }
        lastBButton = gamepad1.b;

        // X re-applies the start pose (robot should be still and at the start spot)
        if (gamepad1.x) {
            setStartPose();
        }

        double targetRelAngleDeg = commandedAngleDeg;
        double distanceToTargetIn = 0.0;

        if (autoAimEnabled) {
            double pivotX = robotX + (TURRET_OFFSET_X_IN * Math.cos(headingRad))
                    - (TURRET_OFFSET_Y_IN * Math.sin(headingRad));
            double pivotY = robotY + (TURRET_OFFSET_X_IN * Math.sin(headingRad))
                    + (TURRET_OFFSET_Y_IN * Math.cos(headingRad));

            double dx = TARGET_X_IN - pivotX;
            double dy = TARGET_Y_IN - pivotY;
            distanceToTargetIn = Math.hypot(dx, dy);

            // Right on top of the target the angle is meaningless, so hold the current aim.
            if (distanceToTargetIn > MIN_AIM_DISTANCE_IN) {
                double fieldAngleDeg = Math.toDegrees(Math.atan2(dy, dx));
                double rawAngleDeg = AngleUnit.normalizeDegrees(fieldAngleDeg - headingDeg + AIM_TRIM_DEG);

                // Pick a reachable angle without flipping sides when the target is behind the robot.
                targetRelAngleDeg = pickReachableAngle(rawAngleDeg, commandedAngleDeg);

                // 1) Low-pass filter toward the target
                double alpha = 1.0 - Math.exp(-dt / SMOOTHING_TAU_SEC);
                double desired = commandedAngleDeg + alpha * (targetRelAngleDeg - commandedAngleDeg);

                // 2) Slew-rate limit
                double maxStep = MAX_SLEW_DEG_PER_SEC * dt;
                double step = Range.clip(desired - commandedAngleDeg, -maxStep, maxStep);

                // 3) Deadband on the real error (not the filtered step)
                if (Math.abs(targetRelAngleDeg - commandedAngleDeg) > DEADBAND_DEG) {
                    commandedAngleDeg += step;
                }
            }
        } else {
            if (gamepad1.dpad_left)  commandedAngleDeg += MANUAL_SPEED_DEG_PER_SEC * dt;
            if (gamepad1.dpad_right) commandedAngleDeg -= MANUAL_SPEED_DEG_PER_SEC * dt;
            targetRelAngleDeg = commandedAngleDeg;
        }

        commandedAngleDeg = Range.clip(commandedAngleDeg, RIGHT_LIMIT_ANGLE_DEG, LEFT_LIMIT_ANGLE_DEG);

        double servoPos = angleToServoPos(commandedAngleDeg);
        sendServo(servoPos);

        telemetry.addData("Mode (B=Toggle, X=ResetPose)", autoAimEnabled ? "AUTO AIM" : "MANUAL (D-Pad L/R)");
        telemetry.addData("Robot Pose", "X: %.1f in | Y: %.1f in | H: %.1f deg", robotX, robotY, headingDeg);
        telemetry.addData("Target Dist", "%.1f in", distanceToTargetIn);
        telemetry.addData("Target Angle", "%.1f deg", targetRelAngleDeg);
        telemetry.addData("Commanded Angle", "%.1f deg", commandedAngleDeg);
        telemetry.addData("Servo Pos", "%.3f", servoPos);
        telemetry.addData("Loop ms", "%.1f", dt * 1000.0);
        telemetry.update();
    }

    private void setStartPose() {
        pinpoint.setPosition(new Pose2D(
                DistanceUnit.INCH, START_X_IN, START_Y_IN,
                AngleUnit.DEGREES, START_HEADING_DEG
        ));
    }

    // If the target is within the turret's range, use it. If it is behind the robot (unreachable),
    // don't snap to the "closer" limit, because that flips at +/-180 and whips the turret across.
    // Instead stay on the side the turret is already on until the target comes back into range.
    private double pickReachableAngle(double rawDeg, double currentDeg) {
        if (rawDeg >= RIGHT_LIMIT_ANGLE_DEG && rawDeg <= LEFT_LIMIT_ANGLE_DEG) {
            return rawDeg;
        }
        return currentDeg >= 0 ? LEFT_LIMIT_ANGLE_DEG : RIGHT_LIMIT_ANGLE_DEG;
    }

    // Only write to the servo when the position meaningfully changes.
    private void sendServo(double pos) {
        if (Math.abs(pos - lastSentServoPos) >= SERVO_WRITE_TOLERANCE) {
            turretServo.setPosition(pos);
            lastSentServoPos = pos;
        }
    }

    private double angleToServoPos(double angleDeg) {
        double clampedAngle = Range.clip(angleDeg, RIGHT_LIMIT_ANGLE_DEG, LEFT_LIMIT_ANGLE_DEG);
        double fraction = (clampedAngle - RIGHT_LIMIT_ANGLE_DEG)
                / (LEFT_LIMIT_ANGLE_DEG - RIGHT_LIMIT_ANGLE_DEG);
        double pos = RIGHT_LIMIT_SERVO_POS + fraction * (LEFT_LIMIT_SERVO_POS - RIGHT_LIMIT_SERVO_POS);
        return Range.clip(pos, 0.0, 1.0);
    }
}//credits claude for smoothening the turret and adding telementary(PWM VALUES)