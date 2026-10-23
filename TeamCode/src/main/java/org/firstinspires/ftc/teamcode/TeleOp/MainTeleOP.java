
package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.RobotInit;
import org.firstinspires.ftc.teamcode.components.DriveSubsystem;
import org.firstinspires.ftc.teamcode.components.intakeSubsystem;
import org.firstinspires.ftc.teamcode.components.ShooterTurretSubsystem;

@TeleOp(name = "Main Multi Mechanism TeleOp", group = "Competition")
public class MainTeleOP extends LinearOpMode {

    private RobotInit config;
    private DriveSubsystem drive;
    private intakeSubsystem intake;
    private ShooterTurretSubsystem shooter;

    @Override
    public void runOpMode() {

        // Set up the robot and all the subsystems
        config = new RobotInit(hardwareMap);

        drive = new DriveSubsystem(config);
        intake = new intakeSubsystem(config);
        shooter = new ShooterTurretSubsystem(config);

        telemetry.addLine("Robot ready");
        telemetry.update();

        waitForStart();

        try {
            while (opModeIsActive()) {

                // Mecanum drive controls
                double forward = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double turn = gamepad1.right_stick_x;

                // Hold left bumper to drive slower
                double driveScale = gamepad1.left_bumper ? 0.4 : 1.0;

                drive.drive(
                        forward * driveScale,
                        strafe * driveScale,
                        turn * driveScale
                );

                // Run both intakes backward to reverse them
                if (gamepad2.dpad_down) {
                    intake.runBoth(-1.0);
                } else {
                    intake.runOuter(gamepad2.left_trigger);
                    intake.runInner(gamepad2.right_trigger);
                }

                // Hold right bumper to spin up the flywheel
                if (gamepad2.right_bumper) {
                    shooter.startFlywheel();
                } else {
                    shooter.stopFlywheel();
                }

                // Use the left stick to turn the turret.bbv
                double turretInput = gamepad2.left_stick_x;

                if (Math.abs(turretInput) > 0.08) {
                    shooter.rotateTurret(turretInput);
                } else {
                    shooter.stopTurret();
                }

                // Move the flower bar up or down
                if (gamepad2.a) {
                    shooter.flowerBarUp();
                } else if (gamepad2.b) {
                    shooter.flowerBarDown();
                }

                // Show what the robot is doing
                telemetry.addData("Forward", "%.2f", forward);
                telemetry.addData("Strafe", "%.2f", strafe);
                telemetry.addData("Turn", "%.2f", turn);
                telemetry.addData("Outer Intake", "%.2f", gamepad2.left_trigger);
                telemetry.addData("Inner Intake", "%.2f", gamepad2.right_trigger);
                telemetry.addData(
                        "Flywheel",
                        gamepad2.right_bumper ? "ON" : "OFF"
                );
                telemetry.addData("Turret", "%.2f", turretInput);
                telemetry.addData(
                        "Flower Bar",
                        gamepad2.a ? "UP" :
                                gamepad2.b ? "DOWN" : "HOLDING"
                );

                telemetry.update();
            }
        } finally {
            // Stop everything when TeleOp ends
            drive.stop();
            intake.stop();
            shooter.stop();
        }
    }
}
