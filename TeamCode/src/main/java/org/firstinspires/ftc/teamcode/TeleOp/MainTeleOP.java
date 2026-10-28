
package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.RobotInit;
import org.firstinspires.ftc.teamcode.components.DriveSubsystem;
import org.firstinspires.ftc.teamcode.components.intakeSubsystem;
import org.firstinspires.ftc.teamcode.components.ShooterTurretSubsystem;

@TeleOp(name = "-BioBuzzTeleOpV1", group = "Competition")
public class MainTeleOP extends LinearOpMode {

    private RobotInit config;
    private DriveSubsystem drive;
    private intakeSubsystem intake;
    private ShooterTurretSubsystem shooter;

    @Override
    public void runOpMode() {

        config = new RobotInit(hardwareMap);

        drive = new DriveSubsystem(config);
        intake = new intakeSubsystem(config);
        shooter = new ShooterTurretSubsystem(config);

        // These remember which mechanisms are switched on.
        boolean outerMiddleOn = false;
        boolean innerOn = false;
        boolean reverseOn = false;
        boolean flywheelOn = false;

        // These remember the previous button states.
        boolean previousA = false;
        boolean previousX = false;
        boolean previousDpadDown = false;
        boolean previousRightBumper = false;

        telemetry.addLine("Robot ready");
        telemetry.addLine("Gamepad 2: Click buttons to toggle mechanisms");
        telemetry.addLine("Left stick up/down: Flower bar position");
        telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive()) {

                // Gamepad 1: Drive controls
                double forward = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double turn = gamepad1.right_stick_x;

                double driveScale = gamepad1.left_bumper ? 0.4 : 1.0;

                drive.drive(
                        forward * driveScale,
                        strafe * driveScale,
                        turn * driveScale,
                        gamepad1.right_trigger,
                        gamepad1.left_trigger
                );

                // Read Gamepad 2 buttons.
                boolean aPressed = gamepad2.a;
                boolean xPressed = gamepad2.x;
                boolean dpadDownPressed = gamepad2.dpad_down;
                boolean rightBumperPressed = gamepad2.right_bumper;

                // A: Toggle outer and middle intakes.
                if (aPressed && !previousA) {
                    outerMiddleOn = !outerMiddleOn;
                }

                // X: Toggle inner intake.
                if (xPressed && !previousX) {
                    innerOn = !innerOn;
                }

                // D-pad down: Toggle reverse for all three intakes.
                if (dpadDownPressed && !previousDpadDown) {
                    reverseOn = !reverseOn;
                }

                // Right bumper: Toggle flywheel.
                if (rightBumperPressed && !previousRightBumper) {
                    flywheelOn = !flywheelOn;
                }

                // Reverse overrides the normal intake settings.
                if (reverseOn) {
                    intake.runBoth(-1.0);
                } else {
                    intake.runOuterAndMiddle(
                            outerMiddleOn ? 1.0 : 0.0
                    );
                    intake.runInner(innerOn ? 1.0 : 0.0);
                }

                // Turn the flywheel on or off.
                if (flywheelOn) {
                    shooter.startFlywheel();
                } else {
                    shooter.stopFlywheel();
                }

                // Gamepad 2 left stick: Set flower bar position.
                // Push down once to move down; release to hold position.
                // Push up once to move up; release to hold position.
                double flowerInput = gamepad2.left_stick_y;

                if (flowerInput > 0.5) {
                    shooter.flowerBarDown();
                } else if (flowerInput < -0.5) {
                    shooter.flowerBarUp();
                }

                // The turret is intentionally not controlled here.

                // Save button states for the next loop.
                previousA = aPressed;
                previousX = xPressed;
                previousDpadDown = dpadDownPressed;
                previousRightBumper = rightBumperPressed;

                // Telemetry
                telemetry.addData(
                        "Drive Power Limit",
                        gamepad1.right_trigger > 0.5 ? "90%" :
                                gamepad1.left_trigger > 0.5 ? "50%" : "70%"
                );

                telemetry.addData(
                        "Outer + Middle",
                        outerMiddleOn ? "ON" : "OFF"
                );
                telemetry.addData(
                        "Inner Intake",
                        innerOn ? "ON" : "OFF"
                );
                telemetry.addData(
                        "Reverse Mode",
                        reverseOn ? "ON" : "OFF"
                );
                telemetry.addData(
                        "Flywheel",
                        flywheelOn ? "ON" : "OFF"
                );

                telemetry.addData(
                        "Outer RPM", "%.0f",
                        intake.getOuterRPM()
                );
                telemetry.addData(
                        "Middle RPM", "%.0f",
                        intake.getMiddleRPM()
                );
                telemetry.addData(
                        "Inner RPM", "%.0f",
                        intake.getInnerRPM()
                );
                telemetry.addData(
                        "Flywheel RPM", "%.0f",
                        shooter.getFlywheelRPM()
                );

                telemetry.update();
            }
        } finally {
            drive.stop();
            intake.stop();
            shooter.stop();
        }
    }
}
