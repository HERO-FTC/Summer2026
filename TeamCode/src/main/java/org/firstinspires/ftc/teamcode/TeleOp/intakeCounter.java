package org.firstinspires.ftc.teamcode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "BioBuzz Intake counter", group = "Test")
@Disabled
public class intakeCounter extends LinearOpMode {
    private DcMotor intake;
    private DigitalChannel laserSensor;
    private static final int MAX_BALLS = 4;
    private int ballCount = 0;
    private boolean previousDetection = false;
    private boolean fullRumbleTriggered = false;
    private double intakePower = 0.50;


    @Override
    public void runOpMode() {

        // Initialize intake motor
        intake = hardwareMap.get(DcMotor.class, "in");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize laser sensor
        laserSensor = hardwareMap.get(DigitalChannel.class, "laserSensor");

        laserSensor.setMode(DigitalChannel.Mode.INPUT);

        // Read initial sensor state
        previousDetection = laserSensor.getState();

        boolean intakeOn = false;
        boolean previousA = false;

        telemetry.addLine("BIOBUZZ INTAKE BALL COUNTER");
        telemetry.addLine("A: Start/Stop Intake");
        telemetry.addData("Maximum Balls", MAX_BALLS);
        telemetry.update();

        waitForStart();

        try {

            while (opModeIsActive()) {

                boolean currentDetection = laserSensor.getState();

                // Detect a new ball (LOW to HIGH)
                if (currentDetection && !previousDetection) {
                    if (ballCount < MAX_BALLS) {
                        ballCount++;
                    }
                }

                previousDetection = currentDetection;

                // Toggle intake using A
                boolean currentA = gamepad1.a;

                if (currentA && !previousA) {
                    intakeOn = !intakeOn;
                }

                previousA = currentA;

                // Stop intake when 4 balls are detected
                if (ballCount >= MAX_BALLS) {

                    intake.setPower(0);
                    intakeOn = false;

                    // Rumble only once
                    if (!fullRumbleTriggered) {
                        gamepad1.rumble(500);
                        fullRumbleTriggered = true;
                    }

                } else if (intakeOn) {

                    intake.setPower(intakePower);

                } else {

                    intake.setPower(0);
                }

                // Driver Station telemetry

                telemetry.addData("Balls Detected", ballCount + " / " + MAX_BALLS);
                telemetry.addData("Laser Detection", currentDetection ? "BALL DETECTED" : "CLEAR");
                telemetry.addData("Intake Status",intakeOn ? "RUNNING" : "STOPPED");
                telemetry.addData("Storage Full", ballCount >= MAX_BALLS);

                telemetry.update();

                idle();
            }

        } finally {

            intake.setPower(0);
        }
    }


}
