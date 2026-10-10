package org.firstinspires.ftc.teamcode.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "New_Path_Blue")
public class Transformation extends OpMode {


    private Follower follower;
    private Timer pathTimer;
    private int pathState;


    private Intake intake;
    private Outtake outtake;




    private final Pose Start_Position = new Pose(21.08708272859215, 120.74818577648766, Math.toRadians(138));


    private final Pose Second_Position = new Pose(57.02685050798257, 83.57619738751816, Math.toRadians(180));
    private final Pose Shoot_Position  = new Pose(52.30333817126269, 88.50507982583456, Math.toRadians(138));
    private final Pose Line1_End = new Pose(27.595791001451367,  82.96008708272862, Math.toRadians(180));
    private final Pose Line2_Start = new Pose( 40.39187227866472,  59.13715529753269, Math.toRadians(180));
    private final Pose Line2_End = new Pose( 27.417271407837436,  59.13715529753269, Math.toRadians(180));


    public static class Intake {
        private final DcMotorEx motor;




        public Intake(HardwareMap hw) {
            motor = hw.get(DcMotorEx.class, "in2");
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        }


        public void set(double mPower) {
            motor.setPower(mPower);


        }
    }


    public static class Outtake{
        private final DcMotorEx o1;
        private final DcMotorEx o2;


        public Outtake(HardwareMap hw){
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


        public void set(double oPower){
            o1.setPower(oPower);
            o2.setPower(oPower);
        }


    }










    private PathChain StartShoot, ShootLine1, Line1End,BackLine1, ToLine2, Line2End, BackLine2;


    public void buildPaths() {
        // Backs Robot up to Shooting Position
        StartShoot = follower.pathBuilder()
                .addPath(new BezierLine(Start_Position, Second_Position))
                .setLinearHeadingInterpolation(Start_Position.getHeading(), Second_Position.getHeading())
                .build();




        ShootLine1 = follower.pathBuilder()
                .addPath(new BezierLine(Second_Position, Line1_End))
                .setConstantHeadingInterpolation(Second_Position.getHeading())
                .build();




        Line1End = follower.pathBuilder()
                .addPath(new BezierLine(Line1_End, Shoot_Position))
                .setLinearHeadingInterpolation(Line1_End.getHeading(), Shoot_Position.getHeading())
                .build();


        BackLine1 = follower.pathBuilder()
                .addPath(new BezierLine(Shoot_Position, Line2_Start))
                .setLinearHeadingInterpolation(Shoot_Position.getHeading(), Line2_Start.getHeading() )
                .build();




        ToLine2 = follower.pathBuilder()
                .addPath(new BezierLine(Line2_Start,Line2_End))
                .setConstantHeadingInterpolation(Line2_Start.getHeading())
                .build();


        Line2End = follower.pathBuilder()
                .addPath(new BezierLine(Line2_End,Shoot_Position))
                .setLinearHeadingInterpolation(Line2_End.getHeading(), Shoot_Position.getHeading())
                .build();
        BackLine2 = follower.pathBuilder()
                .addPath(new BezierLine(Shoot_Position,Start_Position))
                .setLinearHeadingInterpolation(Shoot_Position.getHeading(), Start_Position.getHeading())
                .build();


    }


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                intake.set(0.95);
                outtake.set(0.95);
                follower.followPath(StartShoot);
                setPathState(1);
                break;


            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(ShootLine1);
                    setPathState(2);
                }
                break;


            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(Line1End);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(BackLine1);
                    setPathState(4);
                }
                break;


            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(ToLine2);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(Line2End);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(BackLine2);
                    setPathState(7);
                }
                break;


            case 7:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }


    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }


    @Override
    public void init() {
        pathTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        buildPaths();
        follower.setStartingPose(Start_Position);
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
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}



