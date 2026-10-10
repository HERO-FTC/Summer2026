package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Limelighttargeting {
    private final Limelight3A limelight;
    private double cameraHeightIn = 10.0;
    private double cameraPitchDeg = 25.0;
    private double tagHeightIn = 29.5;

    private double tx = 0.0;
    private double ty = 0.0;
    private boolean valid = false;
    private double distanceIn = 0.0;
    private double lineOfSightIn = 0.0;

    public Limelighttargeting(HardwareMap hardwareMap, String limelightName) {
        limelight = hardwareMap.get(Limelight3A.class, limelightName);
    }

    public void setPipeline(int pipeline) {
        limelight.pipelineSwitch(pipeline);
    }

    public void configure(double cameraHeightIn, double cameraPitchDeg, double tagHeightIn) {
        this.cameraHeightIn = cameraHeightIn;
        this.cameraPitchDeg = cameraPitchDeg;
        this.tagHeightIn = tagHeightIn;
    }

    public void start() {
        limelight.setPollRateHz(100);
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    public boolean update() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            valid = true;
            tx = result.getTx();
            ty = result.getTy();

            double totalAngleRad = Math.toRadians(cameraPitchDeg + ty);
            double denominator = Math.tan(totalAngleRad);
            if (Math.abs(denominator) > 1e-6) {
                distanceIn = (tagHeightIn - cameraHeightIn) / denominator;
                lineOfSightIn = distanceIn / Math.cos(totalAngleRad);
            } else {
                distanceIn = 0.0;
                lineOfSightIn = 0.0;
            }
            return true;
        } else {
            valid = false;
            return false;
        }
    }

    public boolean isValid() {
        return valid;
    }

    public double calibratePitchDeg(double knownDistanceIn) {
        double heightDiff = tagHeightIn - cameraHeightIn;
        if (Math.abs(knownDistanceIn) > 1e-6) {
            double totalAngleRad = Math.atan(heightDiff / knownDistanceIn);
            double totalAngleDeg = Math.toDegrees(totalAngleRad);
            return totalAngleDeg - ty;
        }
        return cameraPitchDeg;
    }

    public void setCameraPitchDeg(double pitch) {
        this.cameraPitchDeg = pitch;
    }

    public double getCameraPitchDeg() {
        return cameraPitchDeg;
    }

    public void setCameraHeightIn(double height) {
        this.cameraHeightIn = height;
    }

    public double getCameraHeightIn() {
        return cameraHeightIn;
    }

    public double getTagHeightIn() {
        return tagHeightIn;
    }

    public double getTx() {
        return tx;
    }

    public double getTy() {
        return ty;
    }

    public double getDistanceIn() {
        return distanceIn;
    }

    public double getLineOfSightIn() {
        return lineOfSightIn;
    }
}
