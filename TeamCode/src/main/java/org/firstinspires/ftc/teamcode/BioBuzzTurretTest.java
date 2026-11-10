
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.CustomMecanumLocalizer;
import org.firstinspires.ftc.teamcode.components.TurretAlignmentSubsystem;
import org.firstinspires.ftc.teamcode.components.TurretServoController;
import org.firstinspires.ftc.teamcode.config.TurretTargetConstants;

@TeleOp(name = "BioBuzz Custom Turret Test")
public class BioBuzzTurretTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        CustomMecanumLocalizer localizer =
                new CustomMecanumLocalizer(hardwareMap);

        TurretServoController servoController =
                new TurretServoController(hardwareMap);

        TurretAlignmentSubsystem turret =
                new TurretAlignmentSubsystem(localizer, servoController);

        boolean aimingEnabled = false;
        boolean previousA = false;
        boolean previousX = false;

        telemetry.addLine("BioBuzz Custom Turret");
        telemetry.addLine("A: Enable or disable aiming");
        telemetry.addLine("X: Reset starting position");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            localizer.update();

            boolean currentA = gamepad1.a;
            boolean currentX = gamepad1.x;

            if (currentA && !previousA) {
                aimingEnabled = !aimingEnabled;
            }

            if (currentX && !previousX) {
                localizer.setPose(
                        TurretTargetConstants.START_X,
                        TurretTargetConstants.START_Y,
                        TurretTargetConstants.START_HEADING
                );
            }

            previousA = currentA;
            previousX = currentX;

            if (aimingEnabled) {
                turret.update();
            } else {
                turret.centerTurret();
            }

            telemetry.addData("Aiming", aimingEnabled);
            telemetry.addData("Robot X", turret.getRobotX());
            telemetry.addData("Robot Y", turret.getRobotY());
            telemetry.addData("Heading", turret.getRobotHeading());
            telemetry.addData("Target X", TurretTargetConstants.TARGET_X);
            telemetry.addData("Target Y", TurretTargetConstants.TARGET_Y);
            telemetry.addData("Target Bearing", turret.getTargetBearing());
            telemetry.addData("Turret Angle", turret.getTargetAngle());
            telemetry.addData("Target Distance", turret.getTargetDistance());
            telemetry.addData(
                    "Servo Position",
                    servoController.getServoPosition()
            );

            telemetry.update();
        }
    }
}
