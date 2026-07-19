package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;

@Autonomous(name = "Spiral In")
public class AD_KB extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    private static final double CENTER_X = 72;
    private static final double CENTER_Y = 72;
    private static final double START_RADIUS = 55;
    private static final double END_RADIUS = 6;
    private static final double TURNS = 3.0;
    private static final int POINTS_PER_TURN = 20;
    private static final boolean SPIN_WHILE_SPIRALING = true;

    private PathChain spiralPath;
    private Pose startPose;

    public void buildPaths() {
        ArrayList<Pose> points = new ArrayList<>();

        double totalAngle = TURNS * 2 * Math.PI;
        int totalPoints = (int) (TURNS * POINTS_PER_TURN);

        for (int i = 0; i <= totalPoints; i++) {
            double t = (double) i / totalPoints;
            double theta = t * totalAngle;
            double radius = START_RADIUS + t * (END_RADIUS - START_RADIUS);

            double x = CENTER_X + radius * Math.cos(theta);
            double y = CENTER_Y + radius * Math.sin(theta);

            double heading = SPIN_WHILE_SPIRALING
                    ? theta % (2 * Math.PI)
                    : Math.toRadians(90);

            points.add(new Pose(x, y, heading));
        }

        startPose = points.get(0);

        PathBuilder builder = follower.pathBuilder();
        for (int i = 0; i < points.size() - 1; i++) {
            Pose from = points.get(i);
            Pose to = points.get(i + 1);
            builder.addPath(new BezierLine(from, to))
                    .setLinearHeadingInterpolation(from.getHeading(), to.getHeading());
        }
        spiralPath = builder.build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(spiralPath);
                setPathState(1);
                break;

            case 1:
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
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }
}
