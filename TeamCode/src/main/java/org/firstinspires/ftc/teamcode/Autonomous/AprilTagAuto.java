
package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.config.RobotInit;
import org.firstinspires.ftc.teamcode.components.intakeSubsystem;
import org.firstinspires.ftc.teamcode.components.ShooterTurretSubsystem;
import org.firstinspires.ftc.teamcode.components.AprilTagAutoSubsystem;

@Autonomous(name = "AprilTag Auto", group = "Competition")
public class AprilTagAuto extends LinearOpMode {

    private AprilTagAutoSubsystem autoSubsystem;

    @Override
    public void runOpMode() {

        // Initialize robot hardware
        RobotInit robot = new RobotInit(hardwareMap);

        // Initialize intake
        intakeSubsystem intake = new intakeSubsystem(robot);

        // Initialize shooter
        ShooterTurretSubsystem shooter =
                new ShooterTurretSubsystem(robot);

        // Initialize autonomous subsystem
        autoSubsystem = new AprilTagAutoSubsystem(
                hardwareMap,
                intake,
                shooter
        );

        telemetry.addLine("AprilTag Auto Initialized");
        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            autoSubsystem.stop();
            return;
        }

        // Start flywheel, intakes, and initial movement
        autoSubsystem.startAuto();

        // Wait for 100ms drive movement to finish
        while (opModeIsActive() &&
                !autoSubsystem.isDriveFinished()) {

            autoSubsystem.update();
        }

        // Enable AprilTag shooting
        autoSubsystem.setShootingEnabled(true);

        // Main autonomous loop
        while (opModeIsActive()) {

            autoSubsystem.update();

            telemetry.addData("Status", "Running");
            telemetry.addData("Shooting", "Enabled");
            telemetry.addData(
                    "Drive Finished",
                    autoSubsystem.isDriveFinished()
            );

            telemetry.update();
        }

        // Stop all mechanisms
        autoSubsystem.stop();
    }
}
