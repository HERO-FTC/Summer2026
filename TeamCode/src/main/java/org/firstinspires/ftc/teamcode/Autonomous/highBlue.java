
package org.firstinspires.ftc.teamcode.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Paths Auto", group = "Pedro Pathing")
public class highBlue extends OpMode {

    private Follower follower;
    private static final double END_TIMEOUT_MS = 500;
    private static final double PATH_MAX_POWER = 0.65;

    private PathChain approach;
    private PathChain returnLeg;
    private boolean returning;

    private final Pose startPose = new Pose(
            82.5318, 141.5, Math.toRadians(90)
    );

    private final Pose point1 = new Pose(
            82.5042, 128.1531, Math.toRadians(270)
    );

    private final Pose point2 = new Pose(
            129.249, 94.4268, Math.toRadians(0)
    );

    private final Pose control2 = new Pose(
            87, 110.88
    );

    private final Pose point3 = new Pose(
            104.3443, 23.9391, Math.toRadians(90)
    );

    private final Pose control3 = new Pose(
            130, 60
    );

    private final Pose point4 = new Pose(
            82.9126, 24.057, Math.toRadians(90)
    );

    private final Pose point5 = new Pose(
            128.6381, 24.1093, Math.toRadians(270)
    );

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {
        follower.update();

        telemetry.addData("Status", "Ready");
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public void start() {

        approach = follower.pathBuilder()

                .addPath(new BezierLine(startPose, point1))
                .setConstantHeadingInterpolation(Math.toRadians(270))

                .addPath(new BezierCurve(point1, control2, point2))
                .setLinearHeadingInterpolation(
                        Math.toRadians(270),
                        Math.toRadians(0)
                )

                .addPath(new BezierCurve(point2, control3, point3))
                .setLinearHeadingInterpolation(
                        Math.toRadians(0),
                        Math.toRadians(90)
                )

                .addPath(new BezierLine(point3, point4))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .setTimeoutConstraint(END_TIMEOUT_MS)
                .build();

        returnLeg = follower.pathBuilder()
                .addPath(new BezierLine(point4, point5))
                .setLinearHeadingInterpolation(
                        Math.toRadians(90),
                        Math.toRadians(270)
                )

                .setTimeoutConstraint(END_TIMEOUT_MS)
                .build();

        returning = false;
        follower.followPath(approach, PATH_MAX_POWER, true);
    }

    @Override
    public void loop() {
        follower.update();

        if (!returning && !follower.isBusy()) {
            returning = true;
            follower.followPath(returnLeg, PATH_MAX_POWER, true);
        }

        telemetry.addData("Status", follower.isBusy()
                ? (returning ? "Returning to point 5" : "Driving to point 4")
                : "Holding point 5");
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));

        telemetry.update();
    }
}
