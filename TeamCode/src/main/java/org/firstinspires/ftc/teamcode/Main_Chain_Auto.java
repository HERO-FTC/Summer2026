package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Main Chain Auto")
public class Main_Chain_Auto extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    private Limelight3A limelight;
    private Turret turret;
    private Outtake outtake;
    private DcMotorEx intake;
    private DcMotorEx intake2;

    private static final int LIMELIGHT_PIPELINE = 9;
    private static final double AIM_DEADBAND_DEG = 2;
    private static final double LONG_TARGET_RPM = 2935;
    private static final long ALIGN_MAX_MS = 3000;
    private static final long FEED_MS = 1250;
    private static final long SHOT_GAP_MS = 150;       // small pause between shots 2/3 (no re-align, no re-windup)
    private static final double FEED_POWER = 0.99;
    private static final double INTAKE_POWER = 0.95;
    private static final double INTAKE2_PICKUP_POWER = 0.75;
    private static final int SHOTS_PER_STOP = 3;

    private boolean feeding = false;
    private long feedStartMs = 0;
    private int shotsFired = 0;
    private boolean finalShotStarted = false; // guards onFinalShotStart firing more than once

    private PathChain path1, path2, path3, path4, path5, path6;
    private Pose startPose;

    public static class Turret {
        private final Servo left, right;

        public Turret(HardwareMap hw) {
            left = hw.get(Servo.class, "l");
            right = hw.get(Servo.class, "r");
        }

        public void center() {
            left.setPosition(0.415);
            right.setPosition(0.415);
        }

        public void custom(double pos) {
            left.setPosition(pos);
            right.setPosition(pos);
        }

        public boolean autoAlign(Limelight3A limelight) {
            LLResult result = limelight.getLatestResult();
            boolean llValid = (result != null && result.isValid());
            double llTxDeg = llValid ? result.getTx() : 0;
            if (llValid && Math.abs(llTxDeg) > AIM_DEADBAND_DEG) {
                double currentPos = right.getPosition();
                double newPos = currentPos + (llTxDeg * 0.002);
                left.setPosition(newPos);
                right.setPosition(newPos);
                return false;
            }
            return llValid;
        }
    }

    public static class Outtake {
        private final DcMotorEx o1, o2;
        static final double OUT_COUNTS_PER_MOTOR_REV = 28;

        public Outtake(HardwareMap hw) {
            o1 = hw.get(DcMotorEx.class, "o1");
            o2 = hw.get(DcMotorEx.class, "o2");
            o1.setDirection(DcMotorEx.Direction.REVERSE);
            o2.setDirection(DcMotorEx.Direction.FORWARD);
            o1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            o2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            o1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            o2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            PIDFCoefficients pidf = new PIDFCoefficients(5, 3, 0, 15.31168224);
            o1.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
            o2.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidf);
        }

        /** Commands the target velocity. The onboard PIDF loop holds it from here. */
        public void spinUp(double targetRPM) {
            double targetTPS = (targetRPM / 60.0) * OUT_COUNTS_PER_MOTOR_REV;
            o1.setVelocity(targetTPS);
            o2.setVelocity(targetTPS);
        }

        public void stop() {
            o1.setVelocity(0);
            o2.setVelocity(0);
        }
    }

    public void buildPaths() {
        startPose = new Pose(118.207, 117.996, Math.toRadians(37));

        path1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(118.207, 117.996), new Pose(82.766, 82.763)))
                .setLinearHeadingInterpolation(Math.toRadians(37), Math.toRadians(46))
                .build();

        path2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(82.766, 82.763), new Pose(119.753, 82.357)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        path3 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(119.753, 82.357), new Pose(83.300, 83.030)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(46))
                .build();

        path4 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(83.300, 83.030), new Pose(83.567, 58.735)))
                .setLinearHeadingInterpolation(Math.toRadians(46), Math.toRadians(0))
                .build();

        path5 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(83.567, 58.735), new Pose(119.460, 58.152)))
                .setTangentHeadingInterpolation()
                .build();

        path6 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(119.460, 58.152), new Pose(83.504, 82.807)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(46))
                .build();
    }

    /**
     * Shoot sequence:
     *  1) SPIN_UP  - command flywheel velocity once, wait until PIDF reports at-speed
     *  2) ALIGN    - turret auto-aligns via Limelight once
     *  3) LAUNCH   - intake + feeder both run continuously to push balls through fast
     *  4) GAP      - tiny pause between shots so the flywheel PID can recover speed,
     *                then immediately back into LAUNCH (no re-align, no re-spin-up)
     * Returns true once all SHOTS_PER_STOP balls are away.
     */
    /**
     * @param onFinalShotStart fires ONCE, the instant the last ball's feed pulse begins -
     *                         use this to kick off the next path immediately so driving
     *                         overlaps with the last shot instead of waiting for it to finish.
     * @param onComplete       fires after the last feed pulse finishes and every shoot motor
     *                         has been stopped - use this to advance pathState.
     */
    private boolean runShootSequence(Runnable onFinalShotStart, Runnable onComplete) {
        outtake.spinUp(LONG_TARGET_RPM); // commanded every loop - PIDF just holds the setpoint, harmless to repeat
        boolean aligned = turret.autoAlign(limelight);
        long elapsed = (long) (pathTimer.getElapsedTimeSeconds() * 1000);

        if (!feeding) {
            // first shot waits on alignment; shots 2 and 3 just need the short recovery gap -
            // no re-align, no re-windup, purely time-based so it can never hang waiting on a
            // sensor condition that might not trip
            boolean readyToFire = (shotsFired == 0)
                    ? (aligned || elapsed >= ALIGN_MAX_MS)
                    : (elapsed >= SHOT_GAP_MS);

            if (readyToFire) {
                feeding = true;
                feedStartMs = elapsed;
                intake.setPower(INTAKE_POWER);   // main intake runs the whole time balls are launching
                intake2.setPower(FEED_POWER);

                if (shotsFired == SHOTS_PER_STOP - 1 && !finalShotStarted) {
                    finalShotStarted = true;
                    if (onFinalShotStart != null) onFinalShotStart.run();
                }
            }
        } else if (elapsed - feedStartMs >= FEED_MS) {
            intake2.setPower(0);
            feeding = false;
            shotsFired++;
            pathTimer.resetTimer();

            if (shotsFired >= SHOTS_PER_STOP) {
                intake.setPower(0);
                outtake.stop();
                shotsFired = 0;
                finalShotStarted = false;
                onComplete.run();
                return true;
            }
        }
        return false;
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(path1);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    turret.center();
                    setPathState(2);
                }
                break;

            case 2: // SHOOT #1-3
                runShootSequence(
                        () -> follower.followPath(path2), // start driving the instant ball #3 begins feeding
                        () -> {
                            intake.setPower(INTAKE_POWER);
                            intake2.setPower(INTAKE2_PICKUP_POWER);
                            setPathState(3);
                        }
                );
                break;

            case 3: // path2 running, intake collecting first row
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    intake2.setPower(0);
                    follower.followPath(path3);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    turret.center();
                    setPathState(5);
                }
                break;

            case 5: // SHOOT #4-6
                runShootSequence(
                        () -> follower.followPath(path4), // start driving the instant ball #6 begins feeding
                        () -> setPathState(6)
                );
                break;

            case 6:
                if (!follower.isBusy()) {
                    intake.setPower(INTAKE_POWER);
                    intake2.setPower(INTAKE2_PICKUP_POWER);
                    follower.followPath(path5);
                    setPathState(7);
                }
                break;

            case 7: // path5 running, intake collecting second row
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    intake2.setPower(0);
                    follower.followPath(path6);
                    setPathState(8);
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    turret.center();
                    setPathState(9);
                }
                break;

            case 9: // SHOOT #7-9
                runShootSequence(null, () -> setPathState(-1)); // last stop, nothing to drive to next
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        feeding = false;
        shotsFired = 0;
        finalShotStarted = false;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        turret = new Turret(hardwareMap);
        outtake = new Outtake(hardwareMap);

        intake = hardwareMap.get(DcMotorEx.class, "in");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake2 = hardwareMap.get(DcMotorEx.class, "in2");
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setDirection(DcMotorEx.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
        limelight.setPollRateHz(100);
        limelight.start();

        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("path state", pathState);
        telemetry.addData("feeding", feeding);
        telemetry.addData("shots fired", shotsFired);
        telemetry.addData("final shot started", finalShotStarted);
        telemetry.addData("follower busy", follower.isBusy());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
