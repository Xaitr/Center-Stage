package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class RedProcessor implements VisionProcessor {
    Telemetry telemetry;

    //This location is returned from the processor
    public enum Location {
        LEFT,
        MIDDLE,
        RIGHT,
        NOT_FOUND
    }
    BlueProcessor.Location location = BlueProcessor.Location.NOT_FOUND;

    //The regions of interest
    //X increases left to right / Y increases up to down
    //x & y give top left corner, width and height go out from there
    private Rect rectRight = new Rect(1140, 500, 220, 250);
    private Rect rectMiddle = new Rect(610, 480, 220, 250);
    private Rect rectLeft = new Rect(60, 500, 220, 250);

    Mat mat = new Mat();

    public RedProcessor(Telemetry telemetry) {this.telemetry = telemetry;}

    public RedProcessor() {}

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGB2HSV);
        Scalar lowHSV = new Scalar(0, 50, 50);
        Scalar highHSV = new Scalar(150, 255, 255);

        Core.inRange(frame, lowHSV, highHSV, frame);

        Mat left = frame.submat(rectLeft);
        Mat right = frame.submat(rectRight);
        Mat middle = frame.submat(rectMiddle);

        double leftValue = Core.sumElems(left).val[0] / rectLeft.area();
        double rightValue = Core.sumElems(right).val[0] / rectRight.area();
        double middleValue = Core.sumElems(middle).val[0] / rectMiddle.area();

        if (telemetry != null) {
            telemetry.addData("Left Value", leftValue);
            telemetry.addData("Right Value", rightValue);
            telemetry.addData("Middle Value", middleValue);
            telemetry.update();
        }

        left.release();
        right.release();
        middle.release();

        if (rightValue > leftValue && rightValue > middleValue) {
            location = BlueProcessor.Location.RIGHT;
            //telemetry.addData("Marker Location", "right");
        }
        else if (leftValue > rightValue && leftValue > middleValue) {
            location = BlueProcessor.Location.LEFT;
            //telemetry.addData("Marker Location", "left");
        } else if (middleValue > rightValue && middleValue > leftValue) {
            location = BlueProcessor.Location.MIDDLE;
            //telemetry.addData("Middle", "Placeholder");
        }
        else {
            location = BlueProcessor.Location.NOT_FOUND;
            //telemetry.addData("Marker Location", "Not Found");
        }


        return location;
    }
    //Converts OpenCV Rectangle to Android rectangle
    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    //Painting :3
    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx,
                            float scaleCanvasDensity, Object userContext) {
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.GREEN);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

        Paint nonSelectedPaint = new Paint(selectedPaint);
        nonSelectedPaint.setColor(Color.RED);

        android.graphics.Rect drawRectangleLeft = makeGraphicsRect(rectLeft, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(rectMiddle, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleRight = makeGraphicsRect(rectRight, scaleBmpPxToCanvasPx);

        location = (BlueProcessor.Location) userContext;
        switch (location) {
            case LEFT:
                canvas.drawRect(drawRectangleLeft, selectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;
            case MIDDLE:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, selectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;
            case RIGHT:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, selectedPaint);
                break;
            case NOT_FOUND:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectangleRight, nonSelectedPaint);
                break;
        }
    }
    public BlueProcessor.Location getLocation() {
        return location;
    }
}
