
package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.config.RobotInit;
import org.firstinspires.ftc.teamcode.components.intakeSubsystem;
import org.firstinspires.ftc.teamcode.components.ShooterTurretSubsystem;
import org.firstinspires.ftc.teamcode.components.AprilTagAutoSubsystem;

@Autonomous(name = "Red AprilTag Auto", group = "Competition")
public class RedAprilTagAuto extends LinearOpMode {

    private AprilTagAutoSubsystem autoSubsystem;

    @Override
    public void runOpMode() {

        RobotInit robot = new RobotInit(hardwareMap);

        intakeSubsystem intake = new intakeSubsystem(robot);

        ShooterTurretSubsystem shooter =
                new ShooterTurretSubsystem(robot);

        autoSubsystem = new AprilTagAutoSubsystem(
                hardwareMap,
                intake,
                shooter,
                false
        );

        telemetry.addLine("Red AprilTag Auto Initialized");
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

            telemetry.addData("Alliance", "RED");
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
