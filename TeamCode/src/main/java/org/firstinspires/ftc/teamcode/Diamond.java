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

@Autonomous(name = "Base Diamond")
public class Diamond extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    // Key heart points (field coords)
    private final Pose bottomTip = new Pose(70.12335216572504, 117.789077212806, Math.toRadians(90));
    private final Pose leftPeak  = new Pose(49.8332628287838, 87.1372856599912, Math.toRadians(90));
    private final Pose topDip    = new Pose(70.35220631908601, 54.02433258469827, Math.toRadians(90));
    private final Pose rightPeak = new Pose(88.9463300755081, 90.13835730368935, Math.toRadians(90));


    private PathChain lobeLeftUp, intoDip, lobeRightUp, backToTip;

    public void buildPaths() {
        // Bottom tip up into the left lobe
        lobeLeftUp = follower.pathBuilder()
                .addPath(new BezierLine(bottomTip, leftPeak))
                .setConstantHeadingInterpolation(bottomTip.getHeading())
                .build();


        intoDip = follower.pathBuilder()
                .addPath(new BezierLine(leftPeak, topDip))
                .setConstantHeadingInterpolation(bottomTip.getHeading())
                .build();


        lobeRightUp = follower.pathBuilder()
                .addPath(new BezierLine(topDip, rightPeak))
                .setConstantHeadingInterpolation(bottomTip.getHeading())
                .build();


        backToTip = follower.pathBuilder()
                .addPath(new BezierLine(rightPeak,bottomTip))
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
