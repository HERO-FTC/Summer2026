
package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.IntakeSubsystemV2;
import org.firstinspires.ftc.teamcode.components.DriveSystemV2;
import org.firstinspires.ftc.teamcode.components.ShooterTurretSubsystemV2;
import org.firstinspires.ftc.teamcode.components.M2TurretAutoAim;
import org.firstinspires.ftc.teamcode.config.RoboInit;

@TeleOp(name = "-BioBuzzTeleOpNEW", group = "Competition")
public class MainTeleOp11_20 extends LinearOpMode {

    @Override
    public void runOpMode() {

        RoboInit robot = new RoboInit(hardwareMap);

        DriveSystemV2 drive = new DriveSystemV2(robot);
        IntakeSubsystemV2 intake = new IntakeSubsystemV2(robot);
        ShooterTurretSubsystemV2 shooter =
                new ShooterTurretSubsystemV2(robot);

        M2TurretAutoAim turret = new M2TurretAutoAim(hardwareMap);

        boolean intakeRunning = false;
        boolean outtakeRunning = false;
        boolean brakeEngaged = false;
        boolean stopperDown = false;
        boolean reverseEverything = false;

        boolean previousA = false;
        boolean previousX = false;
        boolean previousY = false;
        boolean previousRB = false;

        boolean previousDpadDown = false;
        boolean previousDpadRight = false;

        boolean previousStopperDown = false;
        boolean previousStopperUp = false;
        boolean previousFlowerDown = false;

        shooter.setStopper(false);
        shooter.flowerServoUp();

        telemetry.addLine("BioBuzz TeleOp Initialized");
        telemetry.addLine("Stopper: UP");
        telemetry.addLine("Turret Auto Aim: READY");
        telemetry.addLine("Gamepad 1 D-Pad DOWN: Reset to (0, 0)");
        telemetry.addLine("Gamepad 1 D-Pad RIGHT: Reset to (72, 0)");
        telemetry.update();

        waitForStart();
        turret.start();

        try {
            while (opModeIsActive()) {

                double forward = -gamepad1.left_stick_y;
                double strafe = gamepad1.left_stick_x;
                double turn = gamepad1.right_stick_x;

                drive.drive(
                        forward,
                        strafe,
                        turn,
                        gamepad1.right_trigger,
                        gamepad1.left_trigger
                );

                if (gamepad2.a && !previousA) {
                    intakeRunning = !intakeRunning;
                }

                if (gamepad2.x && !previousX) {
                    outtakeRunning = !outtakeRunning;
                }

                if (gamepad2.y && !previousY) {
                    reverseEverything = !reverseEverything;
                }

                // Gamepad 1 D-pad DOWN: Reset to field origin (0, 0)
                if (gamepad1.dpad_down && !previousDpadDown) {
                    turret.resetPosition(0.0, 0.0);
                }

                // Gamepad 1 D-pad RIGHT: Reset to (72, 0)
                else if (gamepad1.dpad_right && !previousDpadRight) {
                    turret.resetPosition(72.0, 0.0);
                }

                if (gamepad2.right_bumper && !previousRB) {
                    brakeEngaged = !brakeEngaged;
                }

                boolean flowerDown = gamepad2.right_trigger > 0.2;

                if (flowerDown != previousFlowerDown) {
                    if (flowerDown) {
                        shooter.flowerServoDown();
                    } else {
                        shooter.flowerServoUp();
                    }

                    previousFlowerDown = flowerDown;
                }

                if (gamepad1.right_bumper && !previousStopperDown) {
                    stopperDown = true;
                }

                if (gamepad1.left_bumper && !previousStopperUp) {
                    stopperDown = false;
                }

                previousA = gamepad2.a;
                previousX = gamepad2.x;
                previousY = gamepad2.y;
                previousRB = gamepad2.right_bumper;

                previousDpadDown = gamepad1.dpad_down;
                previousDpadRight = gamepad1.dpad_right;

                previousStopperDown = gamepad1.right_bumper;
                previousStopperUp = gamepad1.left_bumper;

                if (intakeRunning) {
                    intake.run(reverseEverything ? -1.0 : 1.0);
                } else {
                    intake.stop();
                }

                if (outtakeRunning) {
                    shooter.startFlywheel();
                } else {
                    shooter.stopFlywheel();
                }

                shooter.setBrake(brakeEngaged);
                shooter.setStopper(stopperDown);

                turret.update(telemetry);

                telemetry.addData(
                        "Intake",
                        intakeRunning ?
                                (reverseEverything ? "REVERSE" : "ON") :
                                "OFF"
                );

                telemetry.addData("Outtake", outtakeRunning ? "ON" : "OFF");
                telemetry.addData("Brake", brakeEngaged ? "ENGAGED" : "RELEASED");
                telemetry.addData("Stopper", stopperDown ? "DOWN" : "UP");
                telemetry.addData("Flower Bar", flowerDown ? "DOWN" : "UP");
                telemetry.addData("Intake RPM", "%.1f", intake.getRPM());
                telemetry.addData("Outtake RPM", "%.1f", shooter.getOuttakeRPM());

                telemetry.update();
            }
        } finally {
            shooter.setStopper(false);
            drive.stop();
            intake.stop();
            shooter.stop();
            turret.stop();
        }
    }
}
