package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Limelighttargeting {

    // the 3 numbers the math needs, all in inches / degrees
    // they start at 0 so we HAVE to set them or the distance will be garbage
    private double cameraHeightIn = 0;   // lens to floor
    private double cameraPitchDeg = 0;   // how much the bracket tilts the camera (negative = tilted down)
    private double tagHeightIn    = 0;   // floor to the middle of the tag

    private final Limelight3A limelight; // the actual camera
    private LLResult latest;             // newest frame, gets replaced every update()

    public Limelighttargeting(HardwareMap hw, String name) {
        // name has to match the config on the driver station exactly or it crashes
        limelight = hw.get(Limelight3A.class, name);
    }

    // Mutators
    // these just let us change the numbers from the opmode (handy for tuning)
    public void setCameraHeightIn(double inches)  {
        this.cameraHeightIn = inches;
    }
    public void setCameraPitchDeg(double degrees) {
        this.cameraPitchDeg = degrees;
    }
    public void setTagHeightIn(double inches)     {
        this.tagHeightIn = inches;
    }
    public void setPipeline(int pipeline)         {
        // which pipeline the camera runs (apriltag, yellow pollen, etc)
        limelight.pipelineSwitch(pipeline);
    }


    // sets all 3 at once so we don't need 3 lines in the opmode
    public void configure(double cameraHeightIn, double cameraPitchDeg, double tagHeightIn) {
        setCameraHeightIn(cameraHeightIn);
        setCameraPitchDeg(cameraPitchDeg);
        setTagHeightIn(tagHeightIn);
    }

    // Accessors
    public double getCameraHeightIn() {
        return cameraHeightIn;
    }
    public double getCameraPitchDeg() {
        return cameraPitchDeg;
    }
    public double getTagHeightIn()    {
        return tagHeightIn;
    }

    // Lifecycle
    public void start() {
        // turn the camera on, call this before the loop
        limelight.start();
    }
    public void stop()  {
        // turn it off when we're done
        limelight.stop();
    }


    // call this once every loop, it grabs the newest frame
    // returns true if the camera actually sees something
    public boolean update() {
        latest = limelight.getLatestResult();
        return hasTarget();
    }

    // checks that we have a frame AND it has a real target in it
    // everything below uses this so we don't do math on nothing
    public boolean hasTarget() { return latest != null && latest.isValid(); }

    // Raw angles
    // tx = left/right angle to the tag (negative = left), this is what we aim with
    // ty = up/down angle to the tag (positive = above the center), this is what gives us distance
    // both return 0 if nothing is seen, so check hasTarget() first
    public double getTx() { return hasTarget() ? latest.getTx() : 0; }
    public double getTy() { return hasTarget() ? latest.getTy() : 0; }

    // the real angle from level = bracket tilt + ty
    // needs to be radians because Math.tan / Math.sin don't take degrees
    private double totalAngleRad() {
        return Math.toRadians(cameraPitchDeg + getTy());
    }

    // the actual math

    // distance along the floor to the tag, in inches
    // (tag height - camera height) / tan(angle)
    // gives NaN if no tag or if the angle is basically 0 (can't divide by 0)
    public double getDistanceIn() {
        if (!hasTarget()) return Double.NaN;
        double tan = Math.tan(totalAngleRad());
        if (Math.abs(tan) < 1e-6) return Double.NaN;
        return (tagHeightIn - cameraHeightIn) / tan;
    }


    // straight line from the lens to the tag (the slanted side of the triangle)
    // same idea as above but with sin instead of tan
    public double getLineOfSightIn() {
        if (!hasTarget()) return Double.NaN;
        double sin = Math.sin(totalAngleRad());
        if (Math.abs(sin) < 1e-6) return Double.NaN;
        return (tagHeightIn - cameraHeightIn) / sin;
    }


    // the backwards version: if we already know how far away we are,
    // this gives us how high the tag is (like if the hive tips and the tag drops)
    public double getTargetHeightIn(double knownDistanceIn) {
        if (!hasTarget()) return Double.NaN;
        return cameraHeightIn + knownDistanceIn * Math.tan(totalAngleRad());
    }


    // use this to find the real camera tilt
    // put the robot a measured distance from the tag (use a tape measure),
    // call this, then plug the answer into setCameraPitchDeg()
    public double calibratePitchDeg(double knownDistanceIn) {
        if (!hasTarget()) return Double.NaN;
        double angleToTag = Math.toDegrees(Math.atan2(tagHeightIn - cameraHeightIn, knownDistanceIn));
        return angleToTag - getTy();
    }

    // true if a specific tag id is in the frame
    // so we can ignore the other tags and only care about the goal one
    public boolean seesTag(int id) {
        if (!hasTarget()) return false;
        for (LLResultTypes.FiducialResult f : latest.getFiducialResults()) {
            if (f.getFiducialId() == id) return true;
        }
        return false;
    }
}