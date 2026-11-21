
package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.IntakeSubsystemV2;
import org.firstinspires.ftc.teamcode.components.DriveSystemV2;
import org.firstinspires.ftc.teamcode.components.ShooterTurretSubsystemV2;
import org.firstinspires.ftc.teamcode.config.RoboInit;

@TeleOp(name = "-BioBuzzTeleOpNEW", group = "Competition")
public class MainTeleOp11_20 extends LinearOpMode {

    @Override
    public void runOpMode() {

        RoboInit robot = new RoboInit(hardwareMap);

        DriveSystemV2 drive = new DriveSystemV2(robot);
        IntakeSubsystemV2 intake = new IntakeSubsystemV2(robot);
        ShooterTurretSubsystemV2 shooter = new ShooterTurretSubsystemV2(robot);

        boolean intakeRunning = false;
        boolean outtakeRunning = false;
        boolean brakeEngaged = false;
        boolean stopperDown = false;
        boolean reverseEverything = false;

        boolean previousA = false;
        boolean previousX = false;
        boolean previousY = false;
        boolean previousRB = false;
        boolean previousStopperDown = false;
        boolean previousStopperUp = false;

        telemetry.addLine("BioBuzz TeleOp Initialized");
        telemetry.update();

        waitForStart();

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

                // GAMEPAD 2 A - INTAKE ON/OFF
                if (gamepad2.a && !previousA) {
                    intakeRunning = !intakeRunning;
                }

                // GAMEPAD 2 X - OUTTAKE ON/OFF
                if (gamepad2.x && !previousX) {
                    outtakeRunning = !outtakeRunning;
                }

                // GAMEPAD 2 Y - REVERSE EVERYTHING
                if (gamepad2.y && !previousY) {
                    reverseEverything = !reverseEverything;
                }

                // GAMEPAD 2 RB - BRAKE ON/OFF
                if (gamepad2.right_bumper && !previousRB) {
                    brakeEngaged = !brakeEngaged;
                }

                // GAMEPAD 1 RB - STOPPER DOWN
                if (gamepad1.right_bumper && !previousStopperDown) {
                    stopperDown = true;
                }

                // GAMEPAD 1 LB - STOPPER UP
                if (gamepad1.left_bumper && !previousStopperUp) {
                    stopperDown = false;
                }

                // UPDATE BUTTON STATES
                previousA = gamepad2.a;
                previousX = gamepad2.x;
                previousY = gamepad2.y;
                previousRB = gamepad2.right_bumper;

                previousStopperDown = gamepad1.right_bumper;
                previousStopperUp = gamepad1.left_bumper;

                // INTAKE CONTROL
                if (intakeRunning) {
                    intake.run(reverseEverything ? -1.0 : 1.0);
                } else {
                    intake.stop();
                }

                // OUTTAKE CONTROL
                if (outtakeRunning) {
                    shooter.startOuttake();
                } else {
                    shooter.stopOuttake();
                }

                // BRAKE CONTROL
                shooter.setBrake(brakeEngaged);

                // STOPPER CONTROL
                drive.setStopper(stopperDown);

                // TELEMETRY
                telemetry.addData("Intake", intakeRunning ? "ON" : "OFF");
                telemetry.addData("Outtake", outtakeRunning ? "ON" : "OFF");
                telemetry.addData("Brake", brakeEngaged ? "ENGAGED" : "RELEASED");
                telemetry.addData("Stopper", stopperDown ? "DOWN" : "UP");
                telemetry.addData("Reverse Everything", reverseEverything ? "ON" : "OFF");

                telemetry.addData("Kicker", "RUNNING BACKWARD");
                telemetry.addData("Intake RPM", "%.1f", intake.getRPM());
                telemetry.addData("Outtake RPM", "%.1f", shooter.getOuttakeRPM());

                telemetry.update();
            }

        } finally {

            drive.stop();
            intake.stop();
            shooter.stop();
        }
    }
}

// GAMEPAD 2:
// A - Intake ON/OFF
// X - Outtake ON/OFF
// Y - Reverse everything (motors only, NOT normal servos)
// RB - Brake ON/OFF
//
// GAMEPAD 1:
// RB - Stopper DOWN
// LB - Stopper UP
