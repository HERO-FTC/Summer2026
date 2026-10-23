
package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Base Path")
public class PedroTest1 extends OpMode {

    private Follower follower;
    private Timer pathTimer;
    private int pathState;

    // Starting point and path points
    private final Pose start =
            new Pose(83, 134, Math.toRadians(0));

    private final Pose point2 =
            new Pose(134.8717, 133.713, Math.toRadians(0));

    private final Pose point3 =
            new Pose(134.8236, 96.2074, Math.toRadians(0));

    private PathChain path1;
    private PathChain path1Backward;
    private PathChain path3;
    private PathChain path3Backward;

    public void buildPaths() {

        // Path 1 forward: Start -> Point 2
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(start, point2))
                .setConstantHeadingInterpolation(start.getHeading())
                .build();

        // Path 1 backward: Point 2 -> Start
        path1Backward = follower.pathBuilder()
                .addPath(new BezierLine(point2, start))
                .setConstantHeadingInterpolation(start.getHeading())
                .build();

        // Path 3 forward: Start -> Point 3
        path3 = follower.pathBuilder()
                .addPath(new BezierLine(start, point3))
                .setConstantHeadingInterpolation(start.getHeading())
                .build();

        // Path 3 backward: Point 3 -> Start
        path3Backward = follower.pathBuilder()
                .addPath(new BezierLine(point3, start))
                .setConstantHeadingInterpolation(start.getHeading())
                .build();
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void autonomousPathUpdate() {

        switch (pathState) {

            // 1. Path 1 forward
            case 0:
                follower.followPath(path1);
                setPathState(1);
                break;

            // 2. Path 1 backward
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(path1Backward);
                    setPathState(2);
                }
                break;

            // 3. Path 3 forward
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(path3);
                    setPathState(3);
                }
                break;

            // 4. Path 3 backward
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(path3Backward);
                    setPathState(4);
                }
                break;

            // 5. Finished
            case 4:
                if (!follower.isBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    @Override
    public void init() {
        pathTimer = new Timer();

        follower = Constants.createFollower(hardwareMap);

        buildPaths();

        follower.setStartingPose(start);
    }

    @Override
    public void start() {
        setPathState(0);
    }

    @Override
    public void loop() {
        follower.update();

        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());

        telemetry.addData(
                "Heading (degrees)",
                Math.toDegrees(follower.getPose().getHeading())
        );

        telemetry.addData("Follower Busy", follower.isBusy());

        telemetry.update();
    }
}
