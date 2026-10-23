package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.Set;
@Disabled
public class HiveGate {


    public static double CAM_H_M         = 0.30;   // how high the camera lens sits off the floor (meters)
    public static double CAM_PITCH_DEG   = 25.0;   // how much the camera is tilted up (positive = looking up)
    public static double UP_THRESHOLD_IN = 12.0;   // tag higher than this means the CELL is up (inches)
    public static double STILL_RATE_IPS  = 6.0;    // moving slower than this counts as "holding still" (in/sec)
    public static int    STABLE_FRAMES   = 5;      // how many good frames in a row before we trust it
    public static boolean USE_ROLL_CHECK = true;   // FIRST's trick: upright tag = right-side up CELL
    public static double ROLL_LIMIT_DEG  = 90;     // roll inside +/-90 means upright


    private static final double M_PER_IN = 0.0254;

    private final AprilTagProcessor aprilTag;
    private final Set<Integer> myCellIds;   // only our alliance's tags, so we never aim at the other team's HIVE
    private final Telemetry telemetry;

    // memory from the last frame so we can tell if the CELL is moving
    private double lastHIn = Double.NaN;
    private long lastFrameNs = 0;
    private int stableCount = 0;

    // latest results, mostly for telemetry
    private boolean ready = false;
    private double heightIn = Double.NaN;
    private double rateIps = 0;
    private double rollDeg = Double.NaN;
    private int tagsSeen = 0;

    public HiveGate(AprilTagProcessor aprilTag, Set<Integer> myCellIds, Telemetry telemetry) {
        this.aprilTag = aprilTag;
        this.myCellIds = myCellIds;
        this.telemetry = telemetry;
    }

    public void update() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        double pitch = Math.toRadians(CAM_PITCH_DEG);
        double sumH = 0, sumRoll = 0;
        int n = 0;
        long frameNs = 0;

        for (AprilTagDetection d : detections) {
            // no pose usually means the tag isn't in the tag library, so skip it
            if (d.ftcPose == null) continue;

            // not one of our tags? ignore it
            if (!myCellIds.contains(d.id)) continue;

            // ftcPose: y is how far forward the tag is, z is how far up (relative to the camera)
            double yFwd = d.ftcPose.y;
            double zUp  = d.ftcPose.z;
            double camHIn = CAM_H_M / M_PER_IN;

            // The camera is tilted, so "up" in the camera's view isn't real up.
            // This converts it into actual height above the floor.
            sumH += camHIn + yFwd * Math.sin(pitch) + zUp * Math.cos(pitch);

            sumRoll += d.ftcPose.roll;
            frameNs = Math.max(frameNs, d.frameAcquisitionNanoTime);
            n++;
        }

        tagsSeen = n;

        // can't see any of our tags? then we don't know anything
        if (n == 0) {
            stableCount = 0;
            ready = false;
            return;
        }

        // same frame as last time? nothing new to learn, so don't double count it
        if (frameNs == lastFrameNs) return;

        // average the tags we can see so one noisy tag doesn't throw us off
        heightIn = sumH / n;
        rollDeg = sumRoll / n;

        // how fast is the height changing? big change = the HIVE is tipping
        if (!Double.isNaN(lastHIn) && lastFrameNs != 0) {
            double dt = (frameNs - lastFrameNs) / 1e9;
            if (dt > 1e-3) rateIps = (heightIn - lastHIn) / dt;
        }
        lastHIn = heightIn;
        lastFrameNs = frameNs;

        boolean up = heightIn > UP_THRESHOLD_IN;
        boolean still = Math.abs(rateIps) < STILL_RATE_IPS;
        boolean rollOk = !USE_ROLL_CHECK || Math.abs(rollDeg) < ROLL_LIMIT_DEG;

        // all three have to be true, and one bad frame resets the count to zero
        stableCount = (up && still && rollOk) ? stableCount + 1 : 0;
        ready = stableCount >= STABLE_FRAMES;
    }


    public boolean isReady() {
        return ready;
    }


    public boolean isTipping() {
        return tagsSeen > 0 && Math.abs(rateIps) >= STILL_RATE_IPS;
    }


    public void addTelemetry() {
        telemetry.addData("Hive ready", ready);
        telemetry.addData("Hive tags", tagsSeen);
        telemetry.addData("Hive height (in)", "%.1f", heightIn);
        telemetry.addData("Hive rate (in/s)", "%.1f", rateIps);
        telemetry.addData("Hive roll (deg)", "%.0f", rollDeg);
        telemetry.addData("Hive stable", stableCount);
    }
}