package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="chassisDrive" , group="Linear OpMode")
public class chassisDrive {
    private ElapsedTime runtime = new ElapsedTime();
    static final double TELEOP_DURATION_S = 120.0;   // official TeleOp length
    static final double SHUTDOWN_BEFORE_S  = 0.1;    // shut down 2 seconds early (T = 118s)
    private boolean endgameShutdownApplied = false;  // ensure we only hard-stop once

    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    private double leftFrontPower;
    private double rightFrontPower;
    private double leftBackPower;
    private double rightBackPower;

    public void runOpMode() {
        initHardware();
        waitForStart();
        runtime.reset();
        while (opModeIsActive()){
            handleDriving();
        }
    }

    private boolean opModeIsActive() {
        return false;
    }

    private void waitForStart() {

    }

    private void initHardware() {
        leftFrontDrive = hardwareMap.get(DcMotor.class, "LF");
        leftBackDrive = hardwareMap.get(DcMotor.class, "LB");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RF");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RB");
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    private void handleDriving() {
        double elapsedTeleOp = runtime.seconds();
        boolean endgameActive = elapsedTeleOp >= (TELEOP_DURATION_S - SHUTDOWN_BEFORE_S);
        double max;
        double vertical = gamepad1.left_stick_y;
        double lateral = -gamepad1.left_stick_x;
        double rotate = -gamepad1.right_stick_x;
        leftFrontPower = (vertical + lateral + rotate);
        rightFrontPower = (vertical - lateral - rotate);
        leftBackPower = (vertical - lateral + rotate);
        rightBackPower = (vertical + lateral - rotate);
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }
        if ((gamepad1.right_trigger <= 0) && (gamepad1.left_trigger <=0)) {
            leftFrontPower /= 2;
            rightFrontPower /= 2;
            leftBackPower /= 2;
            rightBackPower /= 2;
        }
        else if ((gamepad1.right_trigger > 0) && (gamepad1.left_trigger == 0)) {
            leftFrontPower = (vertical + lateral + rotate);
            rightFrontPower = (vertical - lateral - rotate);
            leftBackPower = (vertical - lateral + rotate);
            rightBackPower = (vertical + lateral - rotate);
        }
        else if (gamepad1.left_trigger > 0 ) {
            leftFrontPower /= 4;
            rightFrontPower /= 4;
            leftBackPower /= 4;
            rightBackPower /= 4;
        }

        leftFrontDrive.setPower(leftFrontPower);
        rightFrontDrive.setPower(rightFrontPower);
        leftBackDrive.setPower(leftBackPower);
        rightBackDrive.setPower(rightBackPower);

    }
}
