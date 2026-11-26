package org.firstinspires.ftc.teamcode.test;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class ColorBlobPipeline extends OpenCvPipeline {

    public volatile boolean isDetected = false;
    public volatile double distanceCm = 0.0;
    public volatile double angleDeg = 0.0;
    public volatile String targetColor = "NONE";
    public volatile int objectCount = 0;

    private final double frameWidth = 1280;
    private final double focalLength = 850.0;
    private final double knownDiameter = 10.0;
    private final double horizontalFov = 65.0;

    private final double minArea = 550.0;
    private final double minCircularity = 0.60;

    @Override
    public Mat processFrame(Mat input) {
        Mat blurred = new Mat();
        Imgproc.GaussianBlur(input, blurred, new Size(5, 5), 0);

        Mat hsv = new Mat();
        // EasyOpenCV passes RGB/RGBA frames
        Imgproc.cvtColor(blurred, hsv, Imgproc.COLOR_RGB2HSV);

        Scalar lower1 = new Scalar(0, 100, 80);
        Scalar upper1 = new Scalar(12, 255, 255);
        Scalar lower2 = new Scalar(168, 100, 80);
        Scalar upper2 = new Scalar(180, 255, 255);

        Mat mask1 = new Mat();
        Mat mask2 = new Mat();
        Mat mask = new Mat();

        Core.inRange(hsv, lower1, upper1, mask1);
        Core.inRange(hsv, lower2, upper2, mask2);
        Core.bitwise_or(mask1, mask2, mask);

        Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(5, 5));
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_OPEN, kernel);
        Imgproc.morphologyEx(mask, mask, Imgproc.MORPH_CLOSE, kernel);

        List<MatOfPoint> contours = new ArrayList<>();
        Mat hierarchy = new Mat();
        Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        boolean found = false;
        double bestArea = 0;
        MatOfPoint bestContour = null;
        int validCount = 0;

        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            if (area < minArea) continue;

            double perimeter = Imgproc.arcLength(new org.opencv.core.MatOfPoint2f(contour.toArray()), true);
            if (perimeter == 0) continue;

            double circularity = (4 * Math.PI * area) / (perimeter * perimeter);
            if (circularity < minCircularity) continue;

            org.opencv.core.Rect rect = Imgproc.boundingRect(contour);
            double ratio = (double) rect.width / (double) rect.height;
            if (ratio < 0.7 || ratio > 1.4) continue;

            validCount++;

            if (area > bestArea) {
                bestArea = area;
                bestContour = contour;
                found = true;
            }
        }

        objectCount = validCount;

        if (found && bestContour != null) {
            isDetected = true;
            targetColor = "RED";

            Moments M = Imgproc.moments(bestContour);
            if (M.get_m00() != 0) {
                int cx = (int) (M.get_m10() / M.get_m00());
                int cy = (int) (M.get_m01() / M.get_m00());

                org.opencv.core.Rect rect = Imgproc.boundingRect(bestContour);
                double width = rect.width;

                if (width > 0) {
                    distanceCm = (knownDiameter * focalLength) / width;
                }
                double pixelOffset = cx - (frameWidth / 2.0);
                angleDeg = pixelOffset * (horizontalFov / frameWidth);

                Imgproc.drawContours(input, List.of(bestContour), -1, new Scalar(0, 255, 0), 2);
                Imgproc.circle(input, new Point(cx, cy), 5, new Scalar(255, 255, 255), -1);
            }
        } else {
            isDetected = false;
            distanceCm = 0.0;
            angleDeg = 0.0;
            targetColor = "NONE";
        }

        blurred.release();
        hsv.release();
        mask1.release();
        mask2.release();
        mask.release();
        kernel.release();
        hierarchy.release();

        return input;
    }
}