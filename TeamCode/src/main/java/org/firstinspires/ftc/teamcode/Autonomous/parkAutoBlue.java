package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.config.RobotInit;
import org.firstinspires.ftc.teamcode.components.DriveSubsystem;
import org.firstinspires.ftc.teamcode.components.intakeSubsystem;
import org.firstinspires.ftc.teamcode.components.ShooterTurretSubsystem;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.pedro.Constants;


@Autonomous(name = "parkAutoBlue", group = "Competition")
public class parkAutoBlue extends LinearOpMode {

    private static final double SHOOTER_RPM = 4000;
    private static final double SPINUP_TIME = 2.0;
    private static final double BALL_FEED_TIME = 0.8;
    private static final double INTAKE_POWER = 1.0;

    // NOT THE RIGHT VALUES; CHANGE AFTER TUNING
    private static final double START_X = 8;
    private static final double START_Y = 8;
    private static final double START_HEADING = 0;
    private static final double SHOOT_X = 20;
    private static final double SHOOT_Y = 8;
    private static final double SHOOT_HEADING = 0;
    private static final double PARK_X = 20;
    private static final double PARK_Y = 20;
    private static final double PARK_HEADING = 0;

    private static final double DRIVE_TIMEOUT = 6.0;
    private static final double PARK_TIMEOUT = 8.0;

    @Override
    public void runOpMode() {
        RobotInit config = new RobotInit(hardwareMap);
        intakeSubsystem intake = new intakeSubsystem(config);
        ShooterTurretSubsystem shooter = new ShooterTurretSubsystem(config);
        // PLACEHOLDER; CHANGE AFTER TUNING
        Follower follower = Constants.create(hardwareMap);

        Pose startPose = new Pose(
                START_X,
                START_Y,
                Math.toRadians(START_HEADING)
        );
        Pose shootingPose = new Pose (
                SHOOT_X,
                SHOOT_Y,
                Math.toRadians(SHOOT_HEADING)
        );
        Pose parkPose = new Pose (
                PARK_X,
                PARK_Y,
                Math.toRadians(PARK_HEADING)
        );


        telemetry.addLine("BLUE AUTONOMOUS");
        telemetry.addLine("Robot initialized");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }
        try {
            telemetry.addLine("Starting autonomous");
            telemetry.update();

            shooter.setFlywheelRPM(SHOOTER_RPM);
            

            telemetry.addLine("Spinning up flywheel");
            telemetry.update();

            sleep((long) (SPINUP_TIME * 1000));

            telemetry.addLine("Shooting preloaded balls");
            telemetry.update();

            for (int i = 1; i <= 4; i++) {
                if (isStopRequested()) {
                    break;
                }
                telemetry.addData("Shooting ball", i);
                telemetry.update();

                intake.runBoth(INTAKE_POWER);

                sleep((long) (BALL_FEED_TIME * 1000));

                intake.stop();

                sleep(200);
            }

            intake.stop();
            shooter.stopFlywheel();

            telemetry.addLine("Parking");
            telemetry.update();

        } finally {

            intake.stop();
            shooter.stop();
        }
        telemetry.addLine("Autonomous complete");
        telemetry.update();
    }
}
