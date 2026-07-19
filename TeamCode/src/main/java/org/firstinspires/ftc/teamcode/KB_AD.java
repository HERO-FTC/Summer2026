package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Base Pat")
public class KB_AD extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;


    private final Pose bottomTip = new Pose(72, 40, Math.toRadians(0));
    private final Pose leftPeak  = new Pose(40, 95, Math.toRadians(0));
    private final Pose topDip    = new Pose(72, 78, Math.toRadians(0));
    private final Pose rightPeak = new Pose(104, 95, Math.toRadians(0));


    private final Pose zTopLeft    = new Pose(60, 70, Math.toRadians(0));
    private final Pose zTopRight   = new Pose(86, 70, Math.toRadians(0));
    private final Pose zBottomLeft = new Pose(60, 52, Math.toRadians(0));
    private final Pose zBottomRight = new Pose(86, 52, Math.toRadians(0));

    private PathChain lobeLeftUp, intoDip, lobeRightUp, backToTip;
    private PathChain toZStart, zTop, zDiagonal, zBottom;

    public void buildPaths() {

        lobeLeftUp = follower.pathBuilder()
                .addPath(new BezierCurve(bottomTip, new Pose(50, 55), new Pose(30, 80), leftPeak))
                .setConstantHeadingInterpolation(bottomTip.getHeading())
                .build();

        intoDip = follower.pathBuilder()
                .addPath(new BezierCurve(leftPeak, new Pose(50, 100), new Pose(65, 85), topDip))
                .setConstantHeadingInterpolation(bottomTip.getHeading())
                .build();

        lobeRightUp = follower.pathBuilder()
                .addPath(new BezierCurve(topDip, new Pose(79, 85), new Pose(94, 100), rightPeak))
                .setConstantHeadingInterpolation(bottomTip.getHeading())
                .build();

        backToTip = follower.pathBuilder()
                .addPath(new BezierCurve(rightPeak, new Pose(114, 80), new Pose(94, 55), bottomTip))
                .setConstantHeadingInterpolation(bottomTip.getHeading())
                .build();


        toZStart = follower.pathBuilder()
                .addPath(new BezierLine(bottomTip, zTopLeft))
                .setConstantHeadingInterpolation(bottomTip.getHeading())
                .build();


        zTop = follower.pathBuilder()
                .addPath(new BezierLine(zTopLeft, zTopRight))
                .setConstantHeadingInterpolation(bottomTip.getHeading())
                .build();


        zDiagonal = follower.pathBuilder()
                .addPath(new BezierLine(zTopRight, zBottomLeft))
                .setConstantHeadingInterpolation(bottomTip.getHeading())
                .build();


        zBottom = follower.pathBuilder()
                .addPath(new BezierLine(zBottomLeft, zBottomRight))
                .setConstantHeadingInterpolation(bottomTip.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(lobeLeftUp);
                setPathState(1);
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(intoDip);
                    setPathState(2);
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(lobeRightUp);
                    setPathState(3);
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(backToTip);
                    setPathState(4);
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(toZStart);
                    setPathState(5);
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(zTop);
                    setPathState(6);
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(zDiagonal);
                    setPathState(7);
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(zBottom);
                    setPathState(8);
                }
                break;

            case 8:
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
        buildPaths();
        follower.setStartingPose(bottomTip);
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
