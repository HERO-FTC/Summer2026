
package org.firstinspires.ftc.teamcode.localization;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.config.RoboInit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@TeleOp(name = "Blue Turret Calibration", group = "Calibration")
public class BlueTurretCalibration extends LinearOpMode {

    private RoboInit robot;
    private GoBildaPinpointDriver pinpoint;

    private double turretPosition = 0.50;
    private double outtakePower = 0.35;

    private boolean brakeEngaged = false;

    private boolean lastLeft = false;
    private boolean lastRight = false;
    private boolean lastUp = false;
    private boolean lastDown = false;
    private boolean lastCenter = false;
    private boolean lastBrake = false;
    private boolean lastSuccess = false;
    private boolean lastMiss = false;

    private static final double BRAKE_ENGAGED = 0.47;
    private static final double BRAKE_RELEASED = 0.51;

    private static final double TURRET_STEP = 0.01;
    private static final double POWER_STEP = 0.01;

    @Override
    public void runOpMode() {

        robot = new RoboInit(hardwareMap);

        pinpoint = hardwareMap.get(
                GoBildaPinpointDriver.class,
                "pinpoint"
        );

        robot.frontLeft.setDirection(DcMotor.Direction.REVERSE);
        robot.frontRight.setDirection(DcMotor.Direction.REVERSE);
        robot.backLeft.setDirection(DcMotor.Direction.REVERSE);
        robot.backRight.setDirection(DcMotor.Direction.FORWARD);

        robot.frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        robot.intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        robot.outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        configurePinpoint();

        robot.turret.setPosition(turretPosition);
        robot.outtake.setPower(0);
        robot.intake.setPower(0);
        robot.kicker.setPower(0);
        robot.brake.setPosition(BRAKE_RELEASED);

        telemetry.addLine("BLUE TURRET CALIBRATION");
        telemetry.addLine("Hardware initialized.");
        telemetry.addData("Pinpoint", pinpoint.getDeviceStatus());
        telemetry.addLine("Check robot and turret before starting.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            stopRobot();
            return;
        }

        while (opModeIsActive()) {

            driveRobot();

            updateTurret();
            updateOuttake();
            updateIntake();
            updateKicker();
            updateBrake();

            pinpoint.update();

            Pose2D pose = pinpoint.getPosition();

            double x = pose.getX(DistanceUnit.INCH);
            double y = pose.getY(DistanceUnit.INCH);
            double heading = pose.getHeading(AngleUnit.DEGREES);

            updateShotRecording(x, y, heading);
            updateTelemetry(x, y, heading);

            idle();
        }

        stopRobot();
    }

    private void configurePinpoint() {

        pinpoint.setOffsets(
                2.5,
                -4.75,
                DistanceUnit.INCH
        );

        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD
        );

        pinpoint.setEncoderDirections(
                GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD
        );

        pinpoint.resetPosAndIMU();

        pinpoint.setPosition(
                new Pose2D(
                        DistanceUnit.INCH,
                        0,
                        0,
                        AngleUnit.DEGREES,
                        0
                )
        );
    }

    private void driveRobot() {

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double denominator = Math.max(
                Math.abs(forward) + Math.abs(strafe) + Math.abs(turn),
                1.0
        );

        double fl = (forward + strafe + turn) / denominator;
        double fr = (forward - strafe - turn) / denominator;
        double bl = (forward - strafe + turn) / denominator;
        double br = (forward + strafe - turn) / denominator;

        robot.frontLeft.setPower(fl);
        robot.frontRight.setPower(fr);
        robot.backLeft.setPower(bl);
        robot.backRight.setPower(br);
    }

    private void updateTurret() {

        boolean left = gamepad2.dpad_left;
        boolean right = gamepad2.dpad_right;
        boolean center = gamepad2.start;

        if (left && !lastLeft) {
            turretPosition -= TURRET_STEP;
        }

        if (right && !lastRight) {
            turretPosition += TURRET_STEP;
        }

        if (center && !lastCenter) {
            turretPosition = 0.50;
        }

        turretPosition = Range.clip(
                turretPosition,
                0.0,
                1.0
        );

        robot.turret.setPosition(turretPosition);

        lastLeft = left;
        lastRight = right;
        lastCenter = center;
    }

    private void updateOuttake() {

        boolean up = gamepad2.dpad_up;
        boolean down = gamepad2.dpad_down;

        if (up && !lastUp) {
            outtakePower += POWER_STEP;
        }

        if (down && !lastDown) {
            outtakePower -= POWER_STEP;
        }

        outtakePower = Range.clip(
                outtakePower,
                0.0,
                1.0
        );

        robot.outtake.setPower(outtakePower);

        lastUp = up;
        lastDown = down;
    }

    private void updateIntake() {

        robot.intake.setPower(
                gamepad2.left_bumper ? 1.0 : 0.0
        );
    }

    private void updateKicker() {

        robot.kicker.setPower(
                gamepad2.right_bumper ? -1.0 : 0.0
        );
    }

    private void updateBrake() {

        boolean brakeButton = gamepad2.x;

        if (brakeButton && !lastBrake) {
            brakeEngaged = !brakeEngaged;
        }

        robot.brake.setPosition(
                brakeEngaged ? BRAKE_ENGAGED : BRAKE_RELEASED
        );

        lastBrake = brakeButton;
    }

    private void updateShotRecording(
            double x,
            double y,
            double heading
    ) {

        boolean success = gamepad2.y;
        boolean miss = gamepad2.b;

        if (success && !lastSuccess) {
            recordShot(x, y, heading, "SUCCESS");
        }

        if (miss && !lastMiss) {
            recordShot(x, y, heading, "MISS");
        }

        lastSuccess = success;
        lastMiss = miss;
    }

    private void updateTelemetry(
            double x,
            double y,
            double heading
    ) {

        telemetry.addLine("===== BLUE CALIBRATION =====");

        telemetry.addLine("--- LOCALIZATION ---");
        telemetry.addData("Pinpoint Status", pinpoint.getDeviceStatus());
        telemetry.addData("Pinpoint X", "%.2f in", x);
        telemetry.addData("Pinpoint Y", "%.2f in", y);
        telemetry.addData("Heading", "%.2f deg", heading);

        telemetry.addLine("--- TURRET ---");
        telemetry.addData("Servo Position", "%.3f", turretPosition);
        telemetry.addData(
                "Estimated Angle",
                "%.1f deg",
                (turretPosition - 0.5) * 180.0
        );

        telemetry.addLine("--- SHOOTER ---");
        telemetry.addData("Outtake Power", "%.2f", outtakePower);
        telemetry.addData(
                "Outtake Velocity",
                "%.1f ticks/sec",
                robot.outtake.getVelocity()
        );

        telemetry.addLine("--- OTHER MECHANISMS ---");
        telemetry.addData("Intake", "%.2f", robot.intake.getPower());
        telemetry.addData("Kicker", "%.2f", robot.kicker.getPower());
        telemetry.addData("Brake", brakeEngaged ? "ENGAGED" : "RELEASED");

        telemetry.addLine("--- CONTROLS ---");
        telemetry.addLine("G1: Left stick = drive");
        telemetry.addLine("G1: Right stick X = rotate");
        telemetry.addLine("G2: D-pad LEFT/RIGHT = turret");
        telemetry.addLine("G2: D-pad UP/DOWN = outtake power");
        telemetry.addLine("G2: START = turret center");
        telemetry.addLine("G2: LB = intake");
        telemetry.addLine("G2: RB = kicker");
        telemetry.addLine("G2: X = brake toggle");
        telemetry.addLine("G2: Y = record SUCCESS");
        telemetry.addLine("G2: B = record MISS");

        telemetry.update();
    }

    private void recordShot(
            double x,
            double y,
            double heading,
            String result
    ) {

        String row = String.format(
                Locale.US,
                "%.2f,%.2f,%.2f,%.3f,%.3f,%.1f,%s",
                x,
                y,
                heading,
                turretPosition,
                outtakePower,
                robot.outtake.getVelocity(),
                result
        );

        RobotLog.ii("BLUE_CALIBRATION", row);

        telemetry.addData("RECORDED SHOT", row);
    }

    private void stopRobot() {

        if (robot == null) {
            return;
        }

        robot.frontLeft.setPower(0);
        robot.frontRight.setPower(0);
        robot.backLeft.setPower(0);
        robot.backRight.setPower(0);

        robot.intake.setPower(0);
        robot.outtake.setPower(0);
        robot.kicker.setPower(0);
    }
}
