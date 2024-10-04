package org.firstinspires.ftc.teamcode.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;


import java.util.ArrayList;
import java.util.List;

public class BlockDetectorProcessor implements VisionProcessor {

    private Telemetry telemetry;
    private RotatedRect closestBlock;
    private Double angle;

    private enum DetectionMode {
        ALL_COLORS,
        YELLOW_BLUE,
        YELLOW_RED,
        YELLOW_ONLY,
    }

    private DetectionMode detectionMode = DetectionMode.YELLOW_RED;

    public BlockDetectorProcessor(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        // Initialization if needed
    }

    @Override
    public Object processFrame(Mat inputFrame, long captureTimeNanos) {
        Mat bgr = new Mat();
        if (inputFrame.channels() == 4) {
            Imgproc.cvtColor(inputFrame, bgr, Imgproc.COLOR_RGBA2BGR);
        } else {
            bgr = inputFrame;
        }

        Pair<RotatedRect, Double> result = detectBlocks(bgr);
        closestBlock = result.first;
        angle = result.second;

        //drawOutlineAndAngle(bgr, closestBlock, angle);

        // Update telemetry with block information
        double distanceX = 0.0;
        double distanceY = 0.0;
        if (closestBlock != null) {
            double centerX = inputFrame.width() / 2.0;
            double centerY = inputFrame.height() / 2.0;
            distanceX = closestBlock.center.x - centerX;
            distanceY = closestBlock.center.y - centerY;

            telemetry.addData("Distance X", String.format("%.2f pixels", distanceX));
            telemetry.addData("Distance Y", String.format("%.2f pixels", distanceY));
        }
        if (closestBlock != null) {
            telemetry.addData("Closest Block Center", String.format("(%.2f, %.2f)", closestBlock.center.x, closestBlock.center.y));
            if (angle != null) {
                telemetry.addData("Angle", String.format("%.2f degrees", angle));
            }
        } else {
            telemetry.addData("No blocks detected", "");
        }
        telemetry.update();

        return bgr; // Return the processed frame if you want to display it
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        drawOutlineAndAngle(canvas, closestBlock, angle, scaleBmpPxToCanvasPx);
    }

    private Pair<RotatedRect, Double> detectBlocks(Mat image) {
        // Convert the image to HSV color space
        Mat hsvImage = new Mat();
        Imgproc.cvtColor(image, hsvImage, Imgproc.COLOR_BGR2HSV);

        // Define color ranges for red, yellow, and blue blocks (adjust if needed)
        Scalar lowerRed1 = new Scalar(0.0, 100.0, 100.0);
        Scalar upperRed1 = new Scalar(10.0, 255.0, 255.0);
        Scalar lowerRed2 = new Scalar(160.0, 100.0, 100.0);
        Scalar upperRed2 = new Scalar(179.0, 255.0, 255.0);
        Scalar lowerYellow = new Scalar(22.0, 100.0, 100.0);
        Scalar upperYellow = new Scalar(38.0, 255.0, 255.0);
        Scalar lowerBlue = new Scalar(100.0, 100.0, 100.0);
        Scalar upperBlue = new Scalar(120.0, 255.0, 255.0);

        Mat redMask1 = new Mat();
        Mat redMask2 = new Mat();
        Mat yellowMask = new Mat();
        Mat blueMask = new Mat();
        Core.inRange(hsvImage, lowerRed1, upperRed1, redMask1);
        Core.inRange(hsvImage, lowerRed2, upperRed2, redMask2);
        Core.inRange(hsvImage, lowerYellow, upperYellow, yellowMask);
        Core.inRange(hsvImage, lowerBlue, upperBlue, blueMask);

        // Combine red masks
        Mat redMask = new Mat();
        Core.bitwise_or(redMask1, redMask2, redMask);

        // Combine masks based on detection mode
        Mat combinedMask = new Mat();
        switch (detectionMode) {
            case ALL_COLORS:
                Core.bitwise_or(redMask, yellowMask, combinedMask);
                Core.bitwise_or(combinedMask, blueMask, combinedMask);
                break;
            case YELLOW_BLUE:
                Core.bitwise_or(yellowMask, blueMask, combinedMask);
                break;
            case YELLOW_RED:
                Core.bitwise_or(yellowMask, redMask, combinedMask);
                break;
            case YELLOW_ONLY:
                combinedMask = yellowMask;
                break;
        }

        // Convert to grayscale (if combinedMask is not already grayscale)
        Mat gray = new Mat();
        if (combinedMask.channels() > 1) {
            Imgproc.cvtColor(combinedMask, gray, Imgproc.COLOR_BGR2GRAY);
        } else {
            gray = combinedMask;
        }

        // Threshold the image to create a binary image
        Mat binary = new Mat();
        Imgproc.threshold(gray, binary, 1, 255, Imgproc.THRESH_BINARY);

        // Distance transform and thresholding
        Mat distTransform = new Mat();
        Imgproc.distanceTransform(binary, distTransform, Imgproc.DIST_L2, 3);
        Core.normalize(distTransform, distTransform, 0.0, 1.0, Core.NORM_MINMAX);

        Mat sureFg = new Mat();
        Imgproc.threshold(distTransform, sureFg, 0.5, 1.0, Imgproc.THRESH_BINARY);

        // Convert to 8-bit single-channel image
        sureFg.convertTo(sureFg, CvType.CV_8UC1);

        // Find contours of separated blocks
        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(sureFg, contours, new Mat(), Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

        // Find the block closest to the center
        RotatedRect closestBlock = null;
        double closestDistance = Double.MAX_VALUE;
        double centerX = image.width() / 2.0;
        double centerY = image.height() / 2.0;

        for (MatOfPoint contour : contours) {
            RotatedRect rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
            double distance = Math.sqrt(Math.pow(rect.center.x - centerX, 2.0) + Math.pow(rect.center.y - centerY, 2.0));

            if (distance < closestDistance) {
                closestDistance = distance;
                closestBlock = rect;
            }
        }

        // Calculate the angle of the closest block
        Double angle = null;
        if (closestBlock != null) {
            angle = closestBlock.angle;
            if (closestBlock.size.width < closestBlock.size.height) {
                angle += 90;
            }
        }

        return new Pair<>(closestBlock, angle);
    }

    public void setDetectionMode(DetectionMode mode) {
        detectionMode = mode;
    }

    private void drawOutlineAndAngle(Canvas canvas, RotatedRect block, Double angle, float s) {
        if (block != null) {
            Point[] points = new Point[4];
            block.points(points);

            Paint linePaint = new Paint();
            linePaint.setColor(Color.GREEN);
            linePaint.setStrokeWidth(5);

            for (int i = 0; i < 4; i++) {
                canvas.drawLine((float) points[i].x * s, (float) points[i].y * s,
                        (float) points[(i + 1) % 4].x * s, (float) points[(i + 1) % 4].y * s,
                        linePaint);
            }

            if (angle != null) {
                Paint textPaint = new Paint();
                textPaint.setColor(Color.GREEN);
                textPaint.setTextSize(50);

                String text = String.format("%.2f degrees", angle);
                canvas.drawText(text, (float) (block.center.x - 150) * s, (float) (block.center.y - 50) * s, textPaint);
            }
        }
    }

    public RotatedRect getClosestBlock() {
        return closestBlock;
    }

    public Double getAngle() {
        return angle;
    }

    // Helper class for Pair (since Java doesn't have built-in Pair)
    public static class Pair<F, S> {
        public F first;
        public S second;

        public Pair(F first, S second) {
            this.first = first;
            this.second = second;
        }
    }
}