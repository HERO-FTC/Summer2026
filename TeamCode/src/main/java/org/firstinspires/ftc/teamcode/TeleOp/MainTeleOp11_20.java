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
        boolean stopperOpen = false;
        boolean reverseEverything = false;
        boolean kickerRunning = false;
        boolean kickerReversed = false;

        boolean previousA = false;
        boolean previousX = false;
        boolean previousY = false;
        boolean previousRB = false;
        boolean previousB = false;
        boolean previousLB = false;

        // STOPPER BUTTON TRACKING
        boolean previousGamepad1Y = false;

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


                // GAMEPAD 2 B - KICKER FORWARD ON/OFF

                if (gamepad2.b && !previousB) {
                    kickerRunning = !kickerRunning;
                    kickerReversed = false;
                }


                // GAMEPAD 2 LB - KICKER REVERSE ON/OFF

                if (gamepad2.left_bumper && !previousLB) {
                    kickerReversed = !kickerReversed;
                    kickerRunning = false;
                }


                // GAMEPAD 1 Y - STOPPER OPEN/CLOSED

                if (gamepad1.y && !previousGamepad1Y) {
                    stopperOpen = !stopperOpen;
                }


                // UPDATE BUTTON STATES

                previousA = gamepad2.a;
                previousX = gamepad2.x;
                previousY = gamepad2.y;
                previousRB = gamepad2.right_bumper;
                previousB = gamepad2.b;
                previousLB = gamepad2.left_bumper;

                previousGamepad1Y = gamepad1.y;


                // INTAKE CONTROL

                if (intakeRunning) {
                    intake.run(reverseEverything ? -1.0 : 1.0);
                } else {
                    intake.stop();
                }


                // OUTTAKE CONTROL

                if (outtakeRunning) {
                    shooter.startFlywheel();
                } else {
                    shooter.stopFlywheel();
                }


                // KICKER CONTROL

                if (kickerReversed) {
                    shooter.runKicker(-1.0);
                } else if (kickerRunning) {
                    shooter.runKicker(1.0);
                } else {
                    shooter.runKicker(0);
                }


                // BRAKE CONTROL

                shooter.setBrake(brakeEngaged);


                // STOPPER CONTROL

                shooter.setStopper(stopperOpen);


                // TELEMETRY

                telemetry.addData("Intake", intakeRunning ? "ON" : "OFF");
                telemetry.addData("Outtake", outtakeRunning ? "ON" : "OFF");

                telemetry.addData("Brake",
                        brakeEngaged ? "ENGAGED" : "RELEASED");

                telemetry.addData("Stopper",
                        stopperOpen ? "OPEN" : "CLOSED");

                telemetry.addData("Reverse Everything",
                        reverseEverything ? "ON" : "OFF");

                telemetry.addData("Kicker",
                        kickerReversed ? "REVERSING" :
                                kickerRunning ? "RUNNING FORWARD" : "STOPPED");

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
// A  - Intake ON/OFF
// X  - Outtake ON/OFF
// Y  - Reverse Everything
// RB - Brake ON/OFF
// B  - Kicker FORWARD ON/OFF
// LB - Kicker REVERSE ON/OFF
//
// GAMEPAD 1:
// Y  - Stopper OPEN/CLOSED