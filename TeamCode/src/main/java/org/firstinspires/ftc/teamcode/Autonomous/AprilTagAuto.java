
package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.config.RobotInit;
import org.firstinspires.ftc.teamcode.components.intakeSubsystem;
import org.firstinspires.ftc.teamcode.components.ShooterTurretSubsystem;
import org.firstinspires.ftc.teamcode.components.AprilTagAutoSubsystem;

@Autonomous(name = "Blue AprilTag Auto", group = "Competition")
public class AprilTagAuto extends LinearOpMode {

    private AprilTagAutoSubsystem autoSubsystem;

    @Override
    public void runOpMode() {

        // Initialize existing robot configuration
        RobotInit robot = new RobotInit(hardwareMap);

        // Initialize intake
        intakeSubsystem intake = new intakeSubsystem(robot);

        // Initialize shooter
        ShooterTurretSubsystem shooter =
                new ShooterTurretSubsystem(robot);

        // Initialize BLUE autonomous
        autoSubsystem = new AprilTagAutoSubsystem(
                hardwareMap,
                intake,
                shooter,
                true,
                robot
        );

        telemetry.addLine("BLUE AprilTag Auto Initialized");
        telemetry.addLine("Waiting for start...");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            autoSubsystem.stop();
            return;
        }

        autoSubsystem.startAuto();

        while (opModeIsActive() &&
                !autoSubsystem.isDriveFinished()) {

            autoSubsystem.update();
        }

        autoSubsystem.setShootingEnabled(true);

        while (opModeIsActive()) {

            autoSubsystem.update();

            telemetry.addData("Alliance", "BLUE");
            telemetry.addData("Shooting", "Enabled");
            telemetry.addData(
                    "Drive Finished",
                    autoSubsystem.isDriveFinished()
            );

            telemetry.update();
        }

        autoSubsystem.stop();
    }
}
