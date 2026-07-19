package org.firstinspires.ftc.teamcode.Autonomous;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.TeleOP.TurretController;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
/**
 * Pedro Pathing 2.0 conversion of autoBlue_longPARK.
 *
 * IMPORTANT CAVEATS (read before running):
 * 1. RoadRunner's Action/SequentialAction/ParallelAction system has no direct Pedro
 *    equivalent (unless you add the NextFTC command layer on top). This version
 *    replicates the same "drive while flywheel spins up and turret auto-aims" behavior
 *    with a non-blocking state machine driven by follower.update() every loop.
 * 2. The (x, y) values below are copied verbatim from the RoadRunner version. Pedro's
 *    default field convention is typically 0-144 rather than RoadRunner's +/-72
 *    centered-origin convention. Re-tune every pose once you're actually testing
 *    on Pedro's coordinate system.
 * 3. Pedro 2.0 dropped the separate Point class - BezierLine/BezierCurve now take
 *    Pose objects directly.
 * 4. `Constants` here refers to the single Constants.java your project generates
 *    (via Pedro's constants migrator or the 2.0 template) - it is NOT a class from
 *    the Pedro Pathing library itself. Point the import at wherever yours lives.
 */
@Disabled
@Config
@Autonomous(name = "autoBlue_long_PARK_Pedro", group = "Autonomous")
public class BluePark extends LinearOpMode {


    private double intakePwr = 0.95;
    private double servo3_pwr = 0.99;
    static final double LONG_TARGET_RPM_L1 = 3025;
    static final double LONG_TARGET_RPM_L2 = 3025;
    private double deg = 0;

    private final ElapsedTime stateTimer = new ElapsedTime();
    private final ElapsedTime turretTimer = new ElapsedTime();



    public static class Intake {
        private final DcMotorEx motor;
        private final CRServo servo;
        public Intake(HardwareMap hw) {
            motor = hw.get(DcMotorEx.class, "in");
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            servo = hw.get(CRServo.class, "s3");
        }
        public void set(double mPower, double sPower) {
            motor.setPower(mPower);
            servo.setPower(sPower);
        }
    }

    public static class Hood {
        private final Servo servo4;
        public Hood(HardwareMap hw) { servo4 = hw.get(Servo.class, "s4"); }
        public void up() { servo4.setPosition(0.7); }
        public void down() { servo4.setPosition(0.3); }
    }

    public static class Flicker {
        private final Servo servo1, servo2;
        public Flicker(HardwareMap hw) {
            servo1 = hw.get(Servo.class, "s1");
            servo2 = hw.get(Servo.class, "s2");
        }
        public void moveBack() { servo1.setPosition(0.45); servo2.setPosition(0.40); }
        public void moveIn()   { servo1.setPosition(0.25); servo2.setPosition(0.6); }
    }

    public static class Outtake {
        private final DcMotorEx o1, o2;
        static final double OUT_COUNTS_PER_MOTOR_REV = 28;

        public Outtake(HardwareMap hw) {
            o1 = hw.get(DcMotorEx.class, "o1");
            o2 = hw.get(DcMotorEx.class, "o2");
            o1.setDirection(DcMotorEx.Direction.FORWARD);
            o2.setDirection(DcMotorEx.Direction.REVERSE);
            o1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            o2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            o1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            o2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            PIDFCoefficients pidf = new PIDFCoefficients(10, 3, 0, 15.31168224);
            o1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
            o2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
        }
        public void spinUp(double targetRPM) {
            double targetTPS = (targetRPM / 60.0) * OUT_COUNTS_PER_MOTOR_REV;
            o1.setVelocity(targetTPS);
            o2.setVelocity(targetTPS);
        }
        public void stop() { o1.setVelocity(0); o2.setVelocity(0); }
    }


    private static final double AUTO_ALIGN_MAX_S = 5.0;
    private static final double AIM_DEADBAND_DEG = 4;


    private boolean turretAutoAlignStep(TurretController turret) {
        turret.update();

        boolean aimValid = turret.getLlValid();
        double aimErr = Math.abs(turret.getAimErrorDeg());
        boolean aimMet = aimValid && (aimErr <= AIM_DEADBAND_DEG);

        if (aimMet) {
            deg = turret.getCurrentDeg();
            return true;
        }

        telemetry.addData("LL tx", turret.getLlTxDeg());
        telemetry.addData("AimErr", aimErr);
        telemetry.addData("AimOffset", turret.getAimOffsetDeg());
        telemetry.addData("TurretDeg", turret.getCurrentDeg());
        telemetry.addData("TurretPwr", turret.getCurrentPwr());
        telemetry.addData("LLValid", aimValid);
        telemetry.addData("Deg Locked", deg);
        telemetry.update();

        if (turretTimer.seconds() > AUTO_ALIGN_MAX_S) {
            turret.setModeManual();
            turret.customDeg(deg);
            turret.selectIdx();
            telemetry.addLine("AIM TIMEOUT -> Fallback to custom Deg°");
            telemetry.update();
            return true;
        }
        return false;
    }

    private void turretCenterAtEnd(TurretController turret) {
        turret.setModeManual();
        turret.customDeg(deg);
        turret.selectIdx();
        telemetry.addData("TurretPwr", turret.getCurrentPwr());
        telemetry.update();
    }

    private void turretPark(TurretController turret) {
        turret.setModeManual();
        turret.selectCentertDeg();
    }



    private enum State {
        START,
        TO_INITIAL_L,
        CENTER_AFTER_LAUNCH1,
        WAIT_BEFORE_FEED1, FEED1, WAIT_AFTER_FEED1, HOOD_DOWN1,
        TO_CYCLE2,
        STOP_INTAKE_C2,
        RETURN_C2,
        CENTER_AFTER_LAUNCH2,
        WAIT_BEFORE_FEED2, FEED2, WAIT_AFTER_FEED2, HOOD_DOWN2,

        PARK,
        DONE
    }

    private State state = State.START;

    private void goTo(State next) {
        state = next;
        stateTimer.reset();
    }

    @Override
    public void runOpMode() {


        Pose initialPose        = new Pose(64, -10, Math.toRadians(180));
        Pose initialPoseL       = new Pose(58, -10, Math.toRadians(197));
        Pose startPoseAfterSlide= new Pose(58, -10, Math.toRadians(180));
        Pose cycle2Pos          = new Pose(44, -53, Math.toRadians(-90));
        Pose cycle2Return       = new Pose(58, -10, Math.toRadians(195));
        Pose parkPose           = new Pose(60, -40, Math.toRadians(-90));
        double intakeYEnd = -60.0;


        Follower follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(initialPose);

        Intake intake = new Intake(hardwareMap);
        Flicker flicker = new Flicker(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        Hood hood = new Hood(hardwareMap);
        TurretController turretControl = new TurretController("T", "limelight", 9);
        turretControl.turret_init(hardwareMap);
        turretControl.limelight_init(hardwareMap);

        double BLUE_LONG_OFFSET_DEG1 = +3;
        turretControl.setAimOffsetDeg(BLUE_LONG_OFFSET_DEG1);

        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("Init: Waiting for START...");
            telemetry.addData("LLValid", turretControl.getLlValid());
            telemetry.addData("LL tx", turretControl.getLlTxDeg());
            telemetry.addData("AimOffset", turretControl.getAimOffsetDeg());
            telemetry.addData("TurretDeg", turretControl.getCurrentDeg());
            telemetry.update();
        }

        PathChain toInitialL = follower.pathBuilder()
                .addPath(new BezierLine(initialPose, initialPoseL))
                .setLinearHeadingInterpolation(initialPose.getHeading(), initialPoseL.getHeading())
                .build();

        PathChain driveToC2 = follower.pathBuilder()
                .addPath(new BezierCurve(startPoseAfterSlide, cycle2Pos))
                .setLinearHeadingInterpolation(startPoseAfterSlide.getHeading(), cycle2Pos.getHeading())

                .build();

        PathChain returnC2 = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(cycle2Pos.getX(), intakeYEnd, cycle2Pos.getHeading()),
                        cycle2Return))
                .setLinearHeadingInterpolation(cycle2Pos.getHeading(), cycle2Return.getHeading())
                .build();

        PathChain parkAction = follower.pathBuilder()
                .addPath(new BezierCurve(initialPose, parkPose))
                .setLinearHeadingInterpolation(initialPose.getHeading(), parkPose.getHeading())
                .build();

        waitForStart();
        if (isStopRequested()) return;

        goTo(State.START);


        while (opModeIsActive()) {
            follower.update();

            switch (state) {

                case START:
                    hood.up();
                    flicker.moveBack();
                    outtake.spinUp(LONG_TARGET_RPM_L1);
                    follower.followPath(toInitialL);
                    turretTimer.reset();
                    goTo(State.TO_INITIAL_L);
                    break;

                case TO_INITIAL_L: {

                    boolean turretDone = turretAutoAlignStep(turretControl);
                    if (!follower.isBusy() && turretDone) {
                        goTo(State.CENTER_AFTER_LAUNCH1);
                    }
                    break;
                }

                case CENTER_AFTER_LAUNCH1:
                    turretCenterAtEnd(turretControl);
                    goTo(State.WAIT_BEFORE_FEED1);
                    break;

                case WAIT_BEFORE_FEED1:
                    if (stateTimer.seconds() > 5.75) {
                        intake.set(intakePwr, servo3_pwr);
                        goTo(State.FEED1);
                    }
                    break;

                case FEED1:
                    if (stateTimer.seconds() > 0.5) {
                        flicker.moveIn();
                        goTo(State.WAIT_AFTER_FEED1);
                    }
                    break;

                case WAIT_AFTER_FEED1:
                    if (stateTimer.seconds() > 3.0) {
                        hood.down();
                        flicker.moveBack();
                        goTo(State.TO_CYCLE2);
                    }
                    break;

                case TO_CYCLE2:
                    flicker.moveBack();
                    follower.followPath(driveToC2);
                    goTo(State.STOP_INTAKE_C2);
                    break;

                case STOP_INTAKE_C2:
                    if (stateTimer.seconds() > 0.2) {
                        intake.set(0, 0);
                        outtake.spinUp(LONG_TARGET_RPM_L2);
                        hood.up();
                        follower.followPath(returnC2);
                        turretTimer.reset();
                        goTo(State.RETURN_C2);
                    }
                    break;

                case RETURN_C2: {

                    turretCenterAtEnd(turretControl);
                    if (!follower.isBusy()) {
                        goTo(State.CENTER_AFTER_LAUNCH2);
                    }
                    break;
                }

                case CENTER_AFTER_LAUNCH2:
                    turretCenterAtEnd(turretControl);
                    goTo(State.WAIT_BEFORE_FEED2);
                    break;

                case WAIT_BEFORE_FEED2:
                    if (stateTimer.seconds() > 0.5) {
                        intake.set(intakePwr, servo3_pwr);
                        goTo(State.FEED2);
                    }
                    break;

                case FEED2:
                    if (stateTimer.seconds() > 0.25) {
                        flicker.moveIn();
                        goTo(State.WAIT_AFTER_FEED2);
                    }
                    break;

                case WAIT_AFTER_FEED2:
                    if (stateTimer.seconds() > 2.0) {
                        hood.down();
                        flicker.moveBack();
                        goTo(State.PARK);
                    }
                    break;

                case PARK:
                    follower.followPath(parkAction);
                    intake.set(0, 0);
                    outtake.stop();
                    turretPark(turretControl);
                    goTo(State.DONE);
                    break;

                case DONE:
                    if (!follower.isBusy()) {

                    }
                    break;
            }

            telemetry.addData("State", state);
            telemetry.addData("Follower busy", follower.isBusy());
            telemetry.update();
        }
    }
}
